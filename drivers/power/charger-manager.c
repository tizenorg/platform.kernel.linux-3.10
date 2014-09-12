/*
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This driver enables to monitor battery health and control charger
 * during suspend-to-mem.
 * Charger manager depends on other devices. register this later than
 * the depending devices.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
**/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power/charger-manager.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/thermal.h>

/*
 * Default termperature threshold for charging.
 * Every temperature units are in tenth of centigrade.
 */
#define CM_DEFAULT_RECHARGE_TEMP_DIFF	50
#define CM_DEFAULT_CHARGE_TEMP_MAX	500

/*
 * Regard CM_JIFFIES_SMALL jiffies is small enough to ignore for
 * delayed works so that we can run delayed works with CM_JIFFIES_SMALL
 * without any delays.
 */
#define	CM_JIFFIES_SMALL	(2)

/* If y is valid (> 0) and smaller than x, do x = y */
#define CM_MIN_VALID(x, y)	x = (((y > 0) && ((x) > (y))) ? (y) : (x))

/*
 * Regard CM_RTC_SMALL (sec) is small enough to ignore error in invoking
 * rtc alarm. It should be 2 or larger
 */
#define CM_RTC_SMALL		(2)

static LIST_HEAD(cm_list);
static DEFINE_MUTEX(cm_list_mtx);

/* About in-suspend (suspend-again) monitoring */
static struct alarm *cm_timer;

/* Backup RTC alarm time in terms of seconds since 01-01-1970 00:00:00 */
static bool cm_suspended;
static bool cm_timer_set;
static unsigned long cm_suspend_duration_ms;

/* About normal (not suspended) monitoring */
static unsigned long polling_jiffy = ULONG_MAX; /* ULONG_MAX: no polling */
static unsigned long next_polling; /* Next appointed polling time */
static struct workqueue_struct *cm_wq; /* init at driver add */
static struct delayed_work cm_monitor_work; /* init at driver add */

/**
 * is_batt_present - See if the battery presents in place.
 * @cm: charger manager instance.
 */
static bool is_batt_present(struct charger_manager *cm)
{
	struct power_supply *psy;
	union power_supply_propval val;
	bool present = false;
	int i, ret;

	switch (cm->battery_present) {
	case CM_BATTERY_PRESENT:
		present = true;
		break;
	case CM_NO_BATTERY:
		break;
	case CM_FUEL_GAUGE:
		psy = cm->battery.psy;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret == 0 && val.intval)
			present = true;
		break;
	case CM_CHARGER_STAT:
		for (i = 0; i < cm->num_chargers; i++) {
			psy = cm->chargers[i]->psy;
			ret = psy->get_property(psy,
					POWER_SUPPLY_PROP_PRESENT, &val);
			if (ret == 0 && val.intval) {
				present = true;
				break;
			}
		}
		break;
	}

	return present;
}

/**
 * is_ext_pwr_online - See if an external power source is attached to charge
 * @cm: charger manager instance.
 *
 * Returns true if at least one of the chargers of the battery has an external
 * power source attached to charge the battery regardless of whether it is
 * actually charging or not.
 */
static bool is_ext_pwr_online(struct charger_manager *cm)
{
	struct charger_dev *charger;
	int i;

	/* If at least one of them has one, it's yes. */
	for (i = 0; i < cm->num_chargers; i++) {
		charger = cm->chargers[i];
		if (charger->aggr_cc)
			return true;
	}
	return false;
}

/**
 * is_charging - Returns true if the battery is being charged.
 * @cm: charger manager instance.
 */
static bool is_charging(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;

	/* If there is no battery, it cannot be charged */
	if (!is_batt_present(cm))
		return false;

	return battery->status == POWER_SUPPLY_STATUS_CHARGING;
}

/**
 * is_polling_required - Return true if need to continue polling for this CM.
 * @cm: charger manager instance.
 */
static bool is_polling_required(struct charger_manager *cm)
{
	switch (cm->polling_mode) {
	case CM_POLL_DISABLE:
		return false;
	case CM_POLL_ALWAYS:
		return true;
	case CM_POLL_EXTERNAL_POWER_ONLY:
		return is_ext_pwr_online(cm);
	case CM_POLL_CHARGING_ONLY:
		return is_charging(cm);
	default:
		dev_warn(cm->dev, "Incorrect polling_mode (%d)\n",
			 cm->polling_mode);
	}

	return false;
}

/**
 * cm_charger_set_cc - Set charging current.
 * @charger: A charger device instance
 * @cc: Target charging current
 *
 * Whether charging is enabled or not, this will set charging current to given.
 */
static int cm_charger_set_cc(struct charger_dev *charger, int cc)
{
	int ret;

	if (cc < 0)
		return -EINVAL;

	if (cc == charger->curr_cc)
		return 0;

	if (charger->max_cc && cc > charger->max_cc)
		cc = charger->max_cc;

	if (charger->regulator) {
		ret = regulator_set_current_limit(charger->regulator,
						cc, cc);
		if (ret)
			return ret;
	} else {
		if (!charger->psy->set_property)
			return -ENODEV;

		ret = charger->psy->set_property(charger->psy,
				POWER_SUPPLY_PROP_CHARGE_NOW,
				(union power_supply_propval *)&cc);
		if (ret)
			return ret;
	}
	charger->curr_cc = cc;

	return 0;
}

/**
 * cm_charger_enable - Enable the charger to charge.
 * @charger: charger device instance
 */
static int cm_charger_enable(struct charger_dev *charger)
{
	union power_supply_propval val;
	int ret;

	/* Skip enabling if charger's already enabled */
	if (charger->enabled)
		return 0;

	if (charger->regulator) {
		ret = regulator_enable(charger->regulator);
		if (ret)
			return ret;
	} else {
		val.intval = POWER_SUPPLY_STATUS_CHARGING;

		ret = charger->psy->set_property(charger->psy,
					POWER_SUPPLY_PROP_STATUS, &val);
		if (ret)
			return ret;
	}

	charger->charging_start = ktime_to_ms(ktime_get());
	charger->charging_stop = 0;
	charger->enabled = true;

	return 0;
}

/**
 * cm_charger_disable - Disable the charger to charge.
 * @charger: charger device instance
 */
static int cm_charger_disable(struct charger_dev *charger)
{
	union power_supply_propval val;
	int ret;

	/* Skip enabling if charger's already disabled */
	if (!charger->enabled)
		return 0;

	if (charger->regulator) {
		ret = regulator_disable(charger->regulator);
		if (ret)
			return ret;

		if (regulator_is_enabled(charger->regulator))
			regulator_force_disable(charger->regulator);
	} else {
		val.intval = POWER_SUPPLY_STATUS_DISCHARGING;
		ret = charger->psy->set_property(charger->psy,
					POWER_SUPPLY_PROP_STATUS, &val);
		if (ret)
			return ret;
	}
	charger->charging_stop = ktime_to_ms(ktime_get());
	charger->enabled = false;

	return 0;
}

/**
 * cm_set_charging - Try to enable or disable charging.
 * @cm: charger manager instance.
 * @enable: true: enable / false: force_disable
 */
static int cm_set_charging(struct charger_manager *cm, bool enable)
{
	struct battery_entity *battery = &cm->battery;
	struct charger_dev *charger;
	int i, ret;

	if (enable) {
		for (i = 0; i < cm->num_chargers; i++) {
			charger = cm->chargers[i];

			if (charger->externally_control)
				continue;

			cm_charger_set_cc(charger, charger->aggr_cc);

			ret = cm_charger_enable(charger);
			if (ret)
				return ret;
		}
	} else {
		if (battery->fullcharged_at) {
			struct charging_constraints *constraints =
						 &cm->constraints;
			u64 curr = ktime_to_ms(ktime_get());

			if (curr - battery->fullcharged_at <
					 constraints->top_off_time)
				return 0;
		}

		for (i = 0; i < cm->num_chargers; i++) {
			charger = cm->chargers[i];

			if (charger->externally_control)
				continue;

			ret = cm_charger_disable(charger);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/**
 * cm_check_charging_duration - Monitor charging time.
 * @cm: charger manager instance
 *
 * If whole charging duration exceed 'charging_max_duration_ms',
 * cm stop charging to prevent overcharge/overheat.
 */
static bool cm_check_charging_duration(struct charger_manager *cm)
{
	struct charging_constraints *constraints = &cm->constraints;
	struct charger_dev *charger;
	u64 curr = ktime_to_ms(ktime_get());
	u32 duration = 0;
	int i;

	if (!constraints->charging_duration)
		return false;

	if (constraints->charging_avail_at > curr)
		return true;

	constraints->charging_avail_at = 0;

	/* Find the longest charging time */
	for (i = 0; i < cm->num_chargers; i++) {
		int _dur;

		charger = cm->chargers[i];

		if (!charger->enabled)
			continue;

		_dur = curr - charger->charging_start;
		if (_dur > duration)
			duration = _dur;
	}

	if (duration > constraints->charging_duration) {
		dev_dbg(cm->dev, "Charging time exceeds to limit.(%ums:%ums)\n",
			duration, constraints->charging_duration);
		constraints->charging_avail_at =
				curr + constraints->charging_hold_off;
		return true;
	}

	return false;
}

/**
 * update_battery_state - Update current battery state.
 * @battery: battery_entity instance
 *
 * Return true if battery state is varied from last checking.
 */
static bool update_battery_state(struct battery_entity *battery)
{
	struct power_supply *fuelgauge = battery->psy;
	union power_supply_propval val;
	bool updated = false;
	int ret;

	ret = fuelgauge->get_property(fuelgauge,
			POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret && battery->soc != val.intval) {
		battery->soc = val.intval;
		updated = true;
	}

	ret = fuelgauge->get_property(fuelgauge,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret && battery->voltage != val.intval) {
		battery->voltage = val.intval;
		updated = true;
	}

	if (battery->capacity != -EINVAL) {
		ret = fuelgauge->get_property(fuelgauge,
				POWER_SUPPLY_PROP_CHARGE_NOW, &val);
		if (!ret && battery->capacity != val.intval) {
			battery->capacity = val.intval;
			updated = true;
		} else if (ret == -EINVAL) {
			battery->capacity = ret;
		}
	}

	ret = thermal_zone_get_temp(battery->tzd,
			(unsigned long *)&val.intval);
	if (!ret) {
		/* Change unit to decidegree Celcius */
		val.intval /= 100;
		if (battery->temperature != val.intval) {
			battery->temperature = val.intval;
			updated = true;
		}
	}

	return updated;
}

/**
 * cm_check_thermal_stats - Check battery temperature is safe.
 * @cm: charger manager instance.
 *
 * Return 0 if battery temperature is safe, otherwise if temperature
 * reaches out of thermal limitation then return non-zero.
 */
static int cm_check_thermal_status(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;
	struct charging_constraints *constraints = &cm->constraints;
	int upper_limit = constraints->temp_max;
	int lower_limit = constraints->temp_min;

	if (battery->status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
		upper_limit -= constraints->temp_diff;
		lower_limit += constraints->temp_diff;
	}

	if (battery->temperature >= upper_limit) {
		battery->health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return -EPERM;
	}

	if (battery->temperature <= lower_limit) {
		battery->health = POWER_SUPPLY_HEALTH_COLD;
		return -EPERM;
	}

	battery->health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

/**
 * is_full_charged - Decide whether battery is full or not.
 * @cm: charger manager instance.
 */
static bool is_full_charged(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;
	struct charging_constraints *constraints = &cm->constraints;
	int conditions;
	int *threshold;

	if (battery->status == POWER_SUPPLY_STATUS_FULL &&
				constraints->recharging_conds) {
		conditions = constraints->recharging_conds;
		threshold = constraints->recharging_thres;
	} else {
		conditions = constraints->fullbatt_conds;
		threshold = constraints->fullbatt_thres;
	}

	while (conditions) {
		struct power_supply *charger_psy;
		union power_supply_propval val;
		int cond = __fls(conditions);
		int i, ret;

		switch (cond) {
		case CM_FULLBATT_SOC:
			if (battery->soc >= threshold[cond])
				goto fullbatt_ok;
			break;
		case CM_FULLBATT_CAPACITY:
			if (battery->capacity >= threshold[cond])
				goto fullbatt_ok;
			break;
		case CM_FULLBATT_VOLTAGE:
			if (battery->voltage >= threshold[cond])
				goto fullbatt_ok;
			break;
		case CM_FULLBATT_STATUS:
			for (i = 0; i < cm->num_chargers; i++) {
				charger_psy = cm->chargers[i]->psy;
				ret = charger_psy->get_property(charger_psy,
						POWER_SUPPLY_PROP_STATUS, &val);
				if (!ret && val.intval ==
						 POWER_SUPPLY_STATUS_FULL)
					goto fullbatt_ok;
			}
			break;
		default:
			return true;
		}
		conditions &= ~(1 << cond);
	}

	if ((battery->status == POWER_SUPPLY_STATUS_FULL) &&
				constraints->recharging_count) {
		if (++battery->fullbatt_check_count <
				constraints->recharging_count)
			return true;

		battery->fullbatt_check_count = 0;
	}

	return false;

fullbatt_ok:
	if ((battery->status != POWER_SUPPLY_STATUS_FULL) &&
				constraints->fullbatt_count) {
		if (++battery->fullbatt_check_count <
				constraints->fullbatt_count)
			return false;

		battery->fullbatt_check_count = 0;
	}
	return true;
}

/**
 * cm_get_target_status - Check current status and get next target status.
 * @cm: charger manager instance.
 */
static int cm_get_target_status(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;

	if (!is_ext_pwr_online(cm))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	if (cm_check_thermal_status(cm))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	switch (battery->status) {
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_FULL:
		if (cm_check_charging_duration(cm) ||
				is_full_charged(cm)) {
			if (!battery->fullcharged_at)
				battery->fullcharged_at =
				ktime_to_ms(ktime_get());
			return POWER_SUPPLY_STATUS_FULL;
		}
	default:
		break;
	}

	battery->fullcharged_at = 0;

	return POWER_SUPPLY_STATUS_CHARGING;
}

/**
 * _cm_monitor - Monitor a battery and set charging.
 * @cm: charger manager instance.
 */
static int _cm_monitor(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;
	int target;
	bool updated;
	int ret = 0;

	updated = update_battery_state(battery);

	target = cm_get_target_status(cm);

	ret = cm_set_charging(cm, (target == POWER_SUPPLY_STATUS_CHARGING));
	if (ret)
		goto out;

	updated |= (battery->status != target);
	battery->status = target;
out:
	if (updated)
		power_supply_changed(&cm->psy);

	return ret;
}


/**
 * cm_monitor - Monitor every battery.
 */
static bool cm_monitor(void)
{
	bool stop = false;
	struct charger_manager *cm;

	mutex_lock(&cm_list_mtx);

	list_for_each_entry(cm, &cm_list, entry) {
		if (!is_batt_present(cm))
			continue;

		if (_cm_monitor(cm))
			stop = true;
	}

	mutex_unlock(&cm_list_mtx);

	return stop;
}

/**
 * _setup_polling - Setup the next instance of polling.
 * @work: work_struct of the function _setup_polling.
 */
static void _setup_polling(struct work_struct *work)
{
	unsigned long min = ULONG_MAX;
	struct charger_manager *cm;
	bool keep_polling = false;
	unsigned long _next_polling;

	mutex_lock(&cm_list_mtx);

	list_for_each_entry(cm, &cm_list, entry) {
		if (is_polling_required(cm) && cm->polling_interval_ms) {
			keep_polling = true;

			if (min > cm->polling_interval_ms)
				min = cm->polling_interval_ms;
		}
	}

	polling_jiffy = msecs_to_jiffies(min);
	if (polling_jiffy <= CM_JIFFIES_SMALL)
		polling_jiffy = CM_JIFFIES_SMALL + 1;

	if (!keep_polling)
		polling_jiffy = ULONG_MAX;
	if (polling_jiffy == ULONG_MAX)
		goto out;

	WARN(cm_wq == NULL, "charger-manager: workqueue not initialized"
			    ". try it later. %s\n", __func__);

	/*
	 * Use mod_delayed_work() iff the next polling interval should
	 * occur before the currently scheduled one.  If @cm_monitor_work
	 * isn't active, the end result is the same, so no need to worry
	 * about stale @next_polling.
	 */
	_next_polling = jiffies + polling_jiffy;

	if (time_before(_next_polling, next_polling)) {
		mod_delayed_work(cm_wq, &cm_monitor_work, polling_jiffy);
		next_polling = _next_polling;
	} else {
		if (queue_delayed_work(cm_wq, &cm_monitor_work, polling_jiffy))
			next_polling = _next_polling;
	}
out:
	mutex_unlock(&cm_list_mtx);
}
static DECLARE_WORK(setup_polling, _setup_polling);

/**
 * cm_monitor_poller - The Monitor / Poller.
 * @work: work_struct of the function cm_monitor_poller
 *
 * During non-suspended state, cm_monitor_poller is used to poll and monitor
 * the batteries.
 */
static void cm_monitor_poller(struct work_struct *work)
{
	cm_monitor();
	schedule_work(&setup_polling);
}

static int cm_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct charger_manager *cm = container_of(psy,
				struct charger_manager, psy);
	struct battery_entity *battery = &cm->battery;
	int ret = 0;

	update_battery_state(battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (is_batt_present(cm))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery->voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = battery->psy->get_property(battery->psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery->temperature;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!is_batt_present(cm)) {
			/* There is no battery. Assume 100% */
			val->intval = 100;
			break;
		}

		val->intval = battery->soc;

		if (val->intval > 100 ||
			battery->status == POWER_SUPPLY_STATUS_FULL) {
			val->intval = 100;
			break;
		}
		if (val->intval < 0)
			val->intval = 0;

		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (is_ext_pwr_online(cm))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = battery->psy->get_property(battery->psy,
				POWER_SUPPLY_PROP_CHARGE_FULL, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!battery->capacity)
			return -EINVAL;
		val->intval = battery->capacity;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static enum power_supply_property cm_default_props[] = {
	/* Guaranteed to provide */
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property cm_optional_props[] = {
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

#define CM_NUM_OF_PROPS	\
	(ARRAY_SIZE(cm_default_props) + ARRAY_SIZE(cm_optional_props))
#define DEFAULT_CM_PSY_NAME	"battery"

/**
 * cm_setup_timer - For in-suspend monitoring setup wakeup alarm
 *		    for suspend_again.
 *
 * Returns true if the alarm is set for Charger Manager to use.
 * Returns false if
 *	cm_setup_timer fails to set an alarm,
 *	cm_setup_timer does not need to set an alarm for Charger Manager,
 *	or an alarm previously configured is to be used.
 */
static bool cm_setup_timer(void)
{
	struct charger_manager *cm;
	unsigned int wakeup_ms = UINT_MAX;
	int timer_req = 0;

	if (time_after(next_polling, jiffies))
		CM_MIN_VALID(wakeup_ms,
			jiffies_to_msecs(next_polling - jiffies));

	mutex_lock(&cm_list_mtx);
	list_for_each_entry(cm, &cm_list, entry) {
		/* Skip if polling is not required for this CM */
		if (!is_polling_required(cm))
			continue;
		timer_req++;
		if (cm->polling_interval_ms == 0)
			continue;
		CM_MIN_VALID(wakeup_ms, cm->polling_interval_ms);
	}
	mutex_unlock(&cm_list_mtx);

	if (timer_req && cm_timer) {
		ktime_t now, add;

		/*
		 * Set alarm with the polling interval (wakeup_ms)
		 * The alarm time should be NOW + CM_RTC_SMALL or later.
		 */
		if (wakeup_ms == UINT_MAX ||
			wakeup_ms < CM_RTC_SMALL * MSEC_PER_SEC)
			wakeup_ms = 2 * CM_RTC_SMALL * MSEC_PER_SEC;

		pr_info("Charger Manager wakeup timer: %u ms\n", wakeup_ms);

		now = ktime_get_boottime();
		add = ktime_set(0, wakeup_ms * NSEC_PER_MSEC);
		alarm_start(cm_timer, ktime_add(now, add));

		cm_suspend_duration_ms = wakeup_ms;

		return true;
	}
	return false;
}

/**
 * charger_extcon_notifier - receive the state of charger cable
 *			when registered cable is attached or detached.
 *
 * @self: the notifier block of the charger_extcon_notifier.
 * @event: the cable state.
 * @ptr: the data pointer of notifier block.
 */
static int cm_extcon_notifier(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	struct charger_cable *cable =
		container_of(self, struct charger_cable, nb);

	if (event) {
		cable->attached = event;
		cable->charger->aggr_cc += cable->max_uA;
	} else {
		cable->attached = event;
		cable->charger->aggr_cc -= cable->max_uA;
	}

	cancel_delayed_work(&cm_monitor_work);
	queue_delayed_work(cm_wq, &cm_monitor_work, 0);

	return NOTIFY_DONE;
}

/**
 * charger_extcon_init - register external connector to use it
 *			as the charger cable
 *
 * @cm: charger manager instance.
 * @cable: the Charger cable representing the external connector.
 */
static int charger_extcon_init(const char *extcon_name,
				struct charger_cable *cable)
{
	int ret = 0;

	cable->nb.notifier_call = cm_extcon_notifier;
	ret = extcon_register_interest(&cable->extcon_dev, extcon_name,
					cable->name, &cable->nb);
	if (ret < 0) {
		pr_err("Cannot register extcon_dev for %s(cable: %s)\n",
			extcon_name, cable->name);
		return ret;
	}

	cable->attached = extcon_get_cable_state(cable->extcon_dev.edev,
						cable->name);
	return 0;
}

/* help function of sysfs node to control charger(regulator) */
static ssize_t charger_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_dev *charger
		= container_of(attr, struct charger_dev, attr_name);

	return sprintf(buf, "%s\n", charger->psy->name);
}

static ssize_t charger_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_dev *charger
		= container_of(attr, struct charger_dev, attr_state);

	return sprintf(buf, "%s\n", charger->enabled ? "enabled" : "disabled");
}

static ssize_t charger_externally_control_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct charger_dev *charger = container_of(attr,
			struct charger_dev, attr_externally_control);

	return sprintf(buf, "%d\n", charger->externally_control);
}

static ssize_t charger_externally_control_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct charger_dev *charger
		= container_of(attr, struct charger_dev,
					attr_externally_control);
	int ret;
	int externally_control;

	ret = sscanf(buf, "%d", &externally_control);
	if (ret == 0) {
		ret = -EINVAL;
		return ret;
	}

	charger->externally_control = externally_control;

	return count;
}

/**
 * charger_manager_register_sysfs - Register sysfs entry for each charger
 * @cm: charger manager instance.
 *
 * This function add sysfs entry for charger(regulator) to control charger from
 * user-space. If some development board use one more chargers for charging
 * but only need one charger on specific case which is dependent on user
 * scenario or hardware restrictions, the user enter 1 or 0(zero) to '/sys/
 * class/power_supply/battery/charger.[index]/externally_control'. For example,
 * if user enter 1 to 'sys/class/power_supply/battery/charger.[index]/
 * externally_control, this charger isn't controlled from charger-manager and
 * always stay off state of regulator.
 */
static int cm_charger_create_sysfs(struct charger_manager *cm)
{
	struct charger_dev *charger;
	char buf[11];
	char *str;
	int ret = 0;
	int i;

	/* Create sysfs entry to control charger(regulator) */
	for (i = 0; i < cm->num_chargers; i++) {
		charger = cm->chargers[i];

		snprintf(buf, 10, "charger.%d", i);
		str = devm_kzalloc(cm->dev,
				sizeof(char) * (strlen(buf) + 1), GFP_KERNEL);
		if (!str) {
			ret = -ENOMEM;
			goto err;
		}
		strcpy(str, buf);

		charger->attrs[0] = &charger->attr_name.attr;
		charger->attrs[1] = &charger->attr_state.attr;
		charger->attrs[2] = &charger->attr_externally_control.attr;
		charger->attrs[3] = NULL;
		charger->attr_g.name = str;
		charger->attr_g.attrs = charger->attrs;

		sysfs_attr_init(&charger->attr_name.attr);
		charger->attr_name.attr.name = "name";
		charger->attr_name.attr.mode = 0444;
		charger->attr_name.show = charger_name_show;

		sysfs_attr_init(&charger->attr_state.attr);
		charger->attr_state.attr.name = "state";
		charger->attr_state.attr.mode = 0444;
		charger->attr_state.show = charger_state_show;

		sysfs_attr_init(&charger->attr_externally_control.attr);
		charger->attr_externally_control.attr.name
				= "externally_control";
		charger->attr_externally_control.attr.mode = 0644;
		charger->attr_externally_control.show
				= charger_externally_control_show;
		charger->attr_externally_control.store
				= charger_externally_control_store;

		ret = sysfs_create_group(&cm->psy.dev->kobj, &charger->attr_g);
		if (ret < 0) {
			dev_err(cm->dev, "Fail to create sysfs of charger.%d\n",
				i);
			ret = -EINVAL;
			goto err;
		}
	}

err:
	return ret;
}

static struct of_device_id charger_manager_match[] = {
	{
		.compatible = "charger-manager",
	},
	{},
};

static int cm_initialize_data(struct charger_manager *cm)
{
	struct battery_entity *battery = &cm->battery;
	struct charging_constraints *constraints = &cm->constraints;
	struct charger_dev *charger;
	struct device_node *np = cm->dev->of_node;
	struct device_node *constraints_np, *battery_np;
	struct device_node *charger_np, *cable_np;
	const char *buf;
	int ret, idx = 0;

	/* Parse charger manager data */
	if (!of_property_read_string(np, "cm-name", &cm->psy.name))
		cm->psy.name = DEFAULT_CM_PSY_NAME;

	if (of_property_read_u32(np, "cm-poll-mode", &cm->polling_mode))
		cm->polling_mode = CM_POLL_DISABLE;

	of_property_read_u32(np, "cm-poll-interval", &cm->polling_interval_ms);

	if (of_property_read_u32(np, "cm-battery-stat", &cm->battery_present))
		cm->battery_present = CM_BATTERY_PRESENT;


	/* Parse charging contraints */
	constraints_np = of_get_child_by_name(np, "constraints");
	if (!constraints_np)
		return -EINVAL;

	of_property_read_u32(constraints_np, "cm-charging-duration",
					&constraints->charging_duration);

	of_property_read_u32(constraints_np, "cm-charging-hold-off",
					&constraints->charging_hold_off);

	of_property_read_u32(constraints_np, "cm-temp-cold",
					&constraints->temp_min);
	if (of_get_property(constraints_np, "cm-temp-cold-in-minus", NULL))
		constraints->temp_min *= -1;
	of_property_read_u32(constraints_np, "cm-temp-hot",
					&constraints->temp_max);
	of_property_read_u32(constraints_np, "cm-temp-diff",
					&constraints->temp_diff);

	if (!of_property_read_u32(constraints_np, "cm-fullbatt-soc",
			&constraints->fullbatt_thres[CM_FULLBATT_SOC]))
		constraints->fullbatt_conds |= (1 << CM_FULLBATT_SOC);
	if (!of_property_read_u32(constraints_np, "cm-fullbatt-capcity",
			&constraints->fullbatt_thres[CM_FULLBATT_CAPACITY]))
		constraints->fullbatt_conds |= (1 << CM_FULLBATT_CAPACITY);
	if (!of_property_read_u32(constraints_np, "cm-fullbatt-voltage",
			&constraints->fullbatt_thres[CM_FULLBATT_VOLTAGE]))
		constraints->fullbatt_conds |= (1 << CM_FULLBATT_VOLTAGE);
	if (of_get_property(constraints_np, "cm-fullbatt-status", NULL))
		constraints->fullbatt_conds |= (1 << CM_FULLBATT_STATUS);

	if (!constraints->fullbatt_conds)
		return -EINVAL;

	of_property_read_u32(constraints_np, "cm-fullbatt-count",
					&constraints->fullbatt_count);

	of_property_read_u32(constraints_np, "cm-top-off-time",
					&constraints->top_off_time);

	if (!of_property_read_u32(constraints_np, "cm-recharging-soc",
			&constraints->recharging_thres[CM_FULLBATT_SOC]))
		constraints->recharging_conds |= (1 << CM_FULLBATT_SOC);
	if (!of_property_read_u32(constraints_np, "cm-recharging-capcity",
			&constraints->recharging_thres[CM_FULLBATT_CAPACITY]))
		constraints->recharging_conds |= (1 << CM_FULLBATT_CAPACITY);
	if (!of_property_read_u32(constraints_np, "cm-recharging-voltage",
			&constraints->recharging_thres[CM_FULLBATT_VOLTAGE]))
		constraints->recharging_conds |= (1 << CM_FULLBATT_VOLTAGE);
	if (of_get_property(constraints_np, "cm-recharging-status", NULL))
		constraints->recharging_conds |= (1 << CM_FULLBATT_STATUS);

	of_property_read_u32(constraints_np, "cm-recharging-count",
					&constraints->recharging_count);

	/* Parse battery properties */
	battery_np = of_get_child_by_name(np, "battery");
	if (!battery_np)
		return -EINVAL;

	battery->psy = power_supply_get_by_phandle(battery_np, "cm-fuelgauge");
	if (IS_ERR_OR_NULL(battery->psy)) {
		dev_err(cm->dev, "Cannot find fuel gauge\n");
		return -ENODEV;
	}
	if (!of_property_read_string(battery_np, "cm-thermal-zone", &buf))
		battery->tzd = thermal_zone_get_zone_by_name(buf);
	else
		battery->tzd = battery->psy->tzd;
	if (!battery->tzd) {
		dev_err(cm->dev, "No way to monitor battery temperature.\n");
		return -ENODEV;
	}

	/* Parse charger properties */
	cm->num_chargers = of_get_child_count(np);
	/* Except battery, constratints child nodes*/
	cm->num_chargers -= 2;
	cm->chargers = devm_kzalloc(cm->dev,
			 sizeof(cm->chargers) * cm->num_chargers, GFP_KERNEL);
	if (!cm->chargers)
		return -ENOMEM;

	for_each_child_of_node(np, charger_np) {
		if (strncmp(charger_np->name, "charger", 5))
			continue;

		charger = devm_kzalloc(cm->dev, sizeof(*charger), GFP_KERNEL);
		if (!charger)
			return -ENOMEM;

		charger->psy = power_supply_get_by_phandle(charger_np,
							"cm-charger");
		if (IS_ERR_OR_NULL(charger->psy)) {
			dev_err(cm->dev, "Cannot find charger(%s)\n",
							charger_np->name);
			return -ENODEV;
		}

		if (!of_property_read_string(charger_np,
					"cm-regulator-name", &buf)) {
			charger->regulator = regulator_get(cm->dev, buf);
			if (IS_ERR(charger->regulator)) {
				pr_err("Failed to find regulator\n");
				return -ENODEV;
			}
		}

		of_property_read_u32(charger_np, "cm-charger-max-cc",
							&charger->max_cc);

		INIT_LIST_HEAD(&charger->cables);

		/* charger cables */
		for_each_child_of_node(charger_np, cable_np) {
			struct charger_cable *cable;

			cable = devm_kzalloc(cm->dev,
						sizeof(*cable), GFP_KERNEL);
			if (!cable)
				return -ENOMEM;

			of_property_read_string(cable_np,
					"cm-cable-name", &cable->name);

			if (of_property_read_string(cable_np,
						"cm-cable-extcon", &buf))
				return -EINVAL;

			ret = charger_extcon_init(buf, cable);
			if (ret)
				return ret;

			of_property_read_u32(cable_np,
					"cm-cable-max", &cable->max_uA);

			cable->charger = charger;

			if (cable->attached)
				charger->aggr_cc += cable->max_uA;

			list_add(&cable->entry, &charger->cables);
		}
		cm->chargers[idx++] = charger;
	}

	return 0;
}

static void cm_test_and_add_property(struct charger_manager *cm,
					enum power_supply_property psp)
{
	struct power_supply *fuelgauge = cm->battery.psy;
	union power_supply_propval val;

	if (fuelgauge->get_property(fuelgauge, psp, &val))
		return;

	cm->psy.properties[cm->psy.num_properties] = psp;
	cm->psy.num_properties++;
}

static enum alarmtimer_restart cm_timer_func(struct alarm *alarm, ktime_t now)
{
	cm_timer_set = false;
	return ALARMTIMER_NORESTART;
}

static int charger_manager_probe(struct platform_device *pdev)
{
	struct charger_manager *cm;
	struct charger_dev *charger;
	int ret = 0, i = 0;

	cm = devm_kzalloc(&pdev->dev,
			sizeof(struct charger_manager),	GFP_KERNEL);
	if (!cm)
		return -ENOMEM;

	/* Basic Values. Unspecified are Null or 0 */
	cm->dev = &pdev->dev;

	ret = cm_initialize_data(cm);
	if (ret) {
		dev_err(cm->dev, "Failed to initialize charger-manager data\n");
		return ret;
	}

	/* Initialize alarm timer */
	if (alarmtimer_get_rtcdev()) {
		cm_timer = devm_kzalloc(cm->dev, sizeof(*cm_timer), GFP_KERNEL);
		alarm_init(cm_timer, ALARM_BOOTTIME, cm_timer_func);
	}

	if (cm->polling_interval_ms != 0 &&
	    msecs_to_jiffies(cm->polling_interval_ms) <= CM_JIFFIES_SMALL) {
		dev_err(&pdev->dev, "polling_interval_ms is too small\n");
		ret = -EINVAL;
		goto err_out;
	}

	platform_set_drvdata(pdev, cm);

	cm->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	cm->psy.get_property = cm_get_property;
	cm->psy.properties = devm_kzalloc(&pdev->dev,
				sizeof(enum power_supply_property)
				* CM_NUM_OF_PROPS, GFP_KERNEL);
	if (!cm->psy.properties) {
		ret = -ENOMEM;
		goto err_out;
	}

	memcpy(cm->psy.properties, cm_default_props,
			sizeof(enum power_supply_property) *
			ARRAY_SIZE(cm_default_props));
	cm->psy.num_properties = ARRAY_SIZE(cm_default_props);

	/* Find which optional psy-properties are available */
	for (i = 0; i < ARRAY_SIZE(cm_optional_props); i++)
		cm_test_and_add_property(cm, cm_optional_props[i]);

	ret = power_supply_register(NULL, &cm->psy);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register charger-manager(%s)\n",
			cm->psy.name);
		goto err_out;
	}

	/* Register sysfs entry for charger(regulator) */
	ret = cm_charger_create_sysfs(cm);
	if (ret)
		goto err_out;

	/* Add to the list */
	mutex_lock(&cm_list_mtx);
	list_add(&cm->entry, &cm_list);
	mutex_unlock(&cm_list_mtx);

	/*
	 * Charger-manager is capable of waking up the systme from sleep
	 * when event is happend through cm_notify_event()
	 */
	device_init_wakeup(&pdev->dev, true);
	device_set_wakeup_capable(&pdev->dev, false);

	/* Update initial battery state */
	_cm_monitor(cm);

	schedule_work(&setup_polling);

	return 0;

err_out:
	for (i = 0; i < cm->num_chargers; i++) {
		struct charger_cable *cable;

		charger = cm->chargers[i];

		list_for_each_entry(cable, &charger->cables, entry)
			extcon_unregister_interest(&cable->extcon_dev);

		regulator_put(charger->regulator);
	}

	return ret;
}

static int charger_manager_remove(struct platform_device *pdev)
{
	struct charger_manager *cm = platform_get_drvdata(pdev);
	struct charger_dev *charger;
	int i = 0;

	/* Remove from the list */
	mutex_lock(&cm_list_mtx);
	list_del(&cm->entry);
	mutex_unlock(&cm_list_mtx);

	cancel_work_sync(&setup_polling);
	cancel_delayed_work_sync(&cm_monitor_work);

	for (i = 0; i < cm->num_chargers; i++) {
		struct charger_cable *cable;

		charger = cm->chargers[i];

		list_for_each_entry(cable, &charger->cables, entry)
			extcon_unregister_interest(&cable->extcon_dev);

		regulator_put(charger->regulator);
	}

	power_supply_unregister(&cm->psy);

	cm_set_charging(cm, false);

	return 0;
}

static const struct platform_device_id charger_manager_id[] = {
	{ "charger-manager", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, charger_manager_id);

static int cm_suspend_noirq(struct device *dev)
{
	int ret = 0;

	if (device_may_wakeup(dev)) {
		device_set_wakeup_capable(dev, false);
		ret = -EAGAIN;
	}

	return ret;
}

static bool cm_need_to_awake(void)
{
	struct charger_manager *cm;

	if (cm_timer)
		return false;

	mutex_lock(&cm_list_mtx);
	list_for_each_entry(cm, &cm_list, entry) {
		if (is_charging(cm)) {
			mutex_unlock(&cm_list_mtx);
			return true;
		}
	}
	mutex_unlock(&cm_list_mtx);

	return false;
}

static int cm_suspend_prepare(struct device *dev)
{
	if (cm_need_to_awake())
		return -EBUSY;

	if (!cm_suspended)
		cm_suspended = true;

	cm_timer_set = cm_setup_timer();

	if (cm_timer_set) {
		cancel_work_sync(&setup_polling);
		cancel_delayed_work_sync(&cm_monitor_work);
	}

	return 0;
}

static void cm_suspend_complete(struct device *dev)
{
	struct charger_manager *cm = dev_get_drvdata(dev);

	if (cm_suspended)
		cm_suspended = false;

	if (cm_timer_set) {
		ktime_t remain;

		alarm_cancel(cm_timer);
		cm_timer_set = false;
		remain = alarm_expires_remaining(cm_timer);
		cm_suspend_duration_ms -= ktime_to_ms(remain);
		schedule_work(&setup_polling);
	}

	_cm_monitor(cm);

	device_set_wakeup_capable(cm->dev, false);
}

static const struct dev_pm_ops charger_manager_pm = {
	.prepare	= cm_suspend_prepare,
	.suspend_noirq	= cm_suspend_noirq,
	.complete	= cm_suspend_complete,
};

static struct platform_driver charger_manager_driver = {
	.driver = {
		.name = "charger-manager",
		.owner = THIS_MODULE,
		.pm = &charger_manager_pm,
		.of_match_table = charger_manager_match,
	},
	.probe = charger_manager_probe,
	.remove = charger_manager_remove,
	.id_table = charger_manager_id,
};

static int __init charger_manager_init(void)
{
	cm_wq = create_freezable_workqueue("charger_manager");
	INIT_DELAYED_WORK(&cm_monitor_work, cm_monitor_poller);

	return platform_driver_register(&charger_manager_driver);
}
late_initcall(charger_manager_init);

static void __exit charger_manager_cleanup(void)
{
	destroy_workqueue(cm_wq);
	cm_wq = NULL;

	platform_driver_unregister(&charger_manager_driver);
}
module_exit(charger_manager_cleanup);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("Charger Manager");
MODULE_LICENSE("GPL");
