/*
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * MyungJoo.Ham <myungjoo.ham@samsung.com>
 *
 * Charger Manager.
 * This framework enables to control and multiple chargers and to
 * monitor charging even in the context of suspend-to-RAM with
 * an interface combining the chargers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
**/

#ifndef _CHARGER_MANAGER_H
#define _CHARGER_MANAGER_H

#include <linux/power_supply.h>
#include <linux/extcon.h>
#include <linux/alarmtimer.h>

enum data_source {
	CM_BATTERY_PRESENT,
	CM_NO_BATTERY,
	CM_FUEL_GAUGE,
	CM_CHARGER_STAT,
};

enum polling_modes {
	CM_POLL_DISABLE = 0,
	CM_POLL_ALWAYS,
	CM_POLL_EXTERNAL_POWER_ONLY,
	CM_POLL_CHARGING_ONLY,
};

/**
 * struct battery_entity
 * @psy: power_supply instance of fuel gauge device
 * @tzd: Thermal_zone_device instance for battery
 * @soc: Current battery soc
 * @voltage: Current battery voltage
 * @capacity: Current battery capacity
 * @temperature: Current battery temperature
 * @health: Current battery health
 * @status: Current battery status
 * @fullbatt_check_count: Number of times checking if battery is full
 * @fullcharged_at: The time when batter is fully charged
 */
struct battery_entity {
	struct power_supply *psy;
	struct thermal_zone_device *tzd;

	int soc;
	int voltage;
	int capacity;
	int temperature;
	int health;
	int status;

	int fullbatt_check_count;
	u64 fullcharged_at;
};

/**
 * struct charger_cable
 * @name: Cable name
 * @entry: Entry for list
 * @attached: The state of charger cable
 * @max_uA: Maximum current that cable can supply
 * @extcon_dev: extcon device for cable notification
 * @nb: Notification block
 * @charger: The instance of struct charger_dev
 */
struct charger_cable {
	const char *name;
	struct list_head entry;
	bool attached;
	int max_uA;

	struct extcon_specific_cable_nb extcon_dev;
	struct notifier_block nb;

	struct charger_dev *charger;
};

/**
 * struct charger_dev
 * @psy: power_supply instance of charger device
 * @regulator: regulator device instance
 * @enabled: Represent if charger device is enabled
 * @externally_control: Set if charger is externally controlled
 * @max_cc: Maximum charging current that charger device allowed
 * @curr_cc: Charging current
 * @aggr_cc: Aggregated current from conntected cables
 * @cables: List of cables
 * @charging_start: Time of last charging start
 * @charging_stop: Time of last charging end
 * @attr_g: Attribute group for the charger(regulator)
 * @attr_name: "name" sysfs entry
 * @attr_state: "state" sysfs entry
 * @attr_externally_control: "externally_control" sysfs entry
 * @attrs: Arrays pointing to attr_name/state/externally_control for attr_g
 */
struct charger_dev {
	struct power_supply *psy;
	struct regulator *regulator;

	bool enabled;
	int externally_control;

	int max_cc;
	int curr_cc;
	int aggr_cc;

	struct list_head cables;

	u64 charging_start;
	u64 charging_stop;

	struct attribute_group attr_g;
	struct device_attribute attr_name;
	struct device_attribute attr_state;
	struct device_attribute attr_externally_control;
	struct attribute *attrs[4];
};

/**
 * Full battery conditions:
 * - CM_FULLBATT_SOC: Full battery's determined by state-of-charge
 * - CM_FULLBATT_CAPACITY: Full battery's determined by capacity
 * - CM_FULLBATT_VOLTAGE: Full battery's determined by voltage
 * - CM_FULLBATT_STATUS: Full battery's determined by charger status
 */
enum {
	CM_FULLBATT_SOC,
	CM_FULLBATT_CAPACITY,
	CM_FULLBATT_VOLTAGE,
	CM_FULLBATT_STATUS,
};

/**
 * struct charging_constraints
 * @charging_duration: Maximum charging duration
 * @charging_hold_off: Hold off time for expiring of charging duration
 * @charging_avail_at: Represent the time When re-charging is available
 * @temp_min : Minimum battery temperature for charging.
 * @temp_max : Maximum battery temperature for charging.
 * @temp_diff : Temperature diffential to restart charging.
 * @fullbatt_conds: Conditions for determining battery is full
 * @fullbatt_thres: Thresholds for full battery conditions
 * @fullbatt_count: Number of trial for full battery checking
 * @top_off_time: Time for retainning top off mode
 * @recharging_conds: Conditions for determining recharging is required
 * @recharging_thres: Thresholds for recharging conditions
 * @fullbatt_count: Number of trial for recharging checking
 */
struct charging_constraints {
	u32 charging_duration;
	u32 charging_hold_off;
	u32 charging_avail_at;

	int temp_min;
	int temp_max;
	int temp_diff;

	int fullbatt_conds;
	int fullbatt_thres[3];
	int fullbatt_count;

	int top_off_time;

	int recharging_conds;
	int recharging_thres[3];
	int recharging_count;
};

/**
 * struct charger_manager
 * @entry: entry for list
 * @dev: device pointer
 * @psy: power_supply for charger manager
 * @polling_mode: Determine which polling mode will be used
 * @polling_interval_ms: interval in millisecond at which
 * @battery_present:
 *	Specify where information for existance of battery can be obtained
 * @num_chargers: number of chargers associated with the battery
 * @chargers: charger device instances
 * @battery: entity related with battery information
 * @constraints: charging constraint applied to the battery
 */
struct charger_manager {
	struct list_head entry;
	struct device *dev;
	struct power_supply psy;

	enum polling_modes polling_mode;
	unsigned int polling_interval_ms;

	enum data_source battery_present;

	int num_chargers;
	struct charger_dev **chargers;
	struct battery_entity battery;
	struct charging_constraints constraints;
};

#endif /* _CHARGER_MANAGER_H */
