#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/spinlock_types.h>
#include <linux/threads.h>
#include <linux/cred.h>
#include <linux/security.h>

#include <linux/configfs.h>


DEFINE_SPINLOCK(pcc_lock);

static char *arg_command_filter = "";
static int arg_cap_cumulative = 1;

module_param(arg_command_filter, charp, 0000);
MODULE_PARM_DESC(arg_command_filter, "Command filter:\n\tempty:\tall\n\t<command_name>:\tcommand specified");

module_param(arg_cap_cumulative, int, 0000);
MODULE_PARM_DESC(arg_cap_cumulative, "Capabilities mode:\n\t0:\tcurrent\n\t1:\tcumulative");

struct process_cap_comm{
	int cap_mask;
};

struct process_filter_opts{
	struct configfs_subsystem subsys;

	char command_filter[TASK_COMM_LEN+1];
	unsigned int cap_cumulative;

	struct process_cap_comm pcc_array[PID_MAX_DEFAULT];
};

static inline struct process_filter_opts *to_process_filter_opts(struct config_item *item){
	return item ? container_of(to_configfs_subsystem(
		to_config_group(item)), struct process_filter_opts, subsys) :
		NULL;
}

CONFIGFS_ATTR_STRUCT(process_filter_opts);
CONFIGFS_ATTR_OPS(process_filter_opts);

static void process_filter_opts_attr_release(struct config_item *item){
}

static struct configfs_item_operations process_filter_item_opts = {
	.release = process_filter_opts_attr_release,
	.show_attribute = process_filter_opts_attr_show,
	.store_attribute = process_filter_opts_attr_store,
};

static ssize_t process_filter_opts_cap_cumulative_show(struct process_filter_opts *opts, char *page){
	unsigned long flags;
	ssize_t len;
	spin_lock_irqsave(&pcc_lock, flags);
	len = sprintf(page, "%d\n", opts->cap_cumulative);
	spin_unlock_irqrestore(&pcc_lock, flags);
	return len;
}

static ssize_t process_filter_opts_cap_cumulative_store(struct process_filter_opts *opts, const char *page, size_t count){
	unsigned long flags;
	unsigned long val;
	int ret;
	unsigned int i;

	ret = strict_strtoul(page, 0, &val);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&pcc_lock, flags);
	if (val == 0){
		opts->cap_cumulative=0;
	}else{
		if (opts->cap_cumulative==0){
			for(i=0; i<PID_MAX_DEFAULT; ++i){
				opts->pcc_array[i].cap_mask=0;
			}
			opts->cap_cumulative=1;
		}
	}
	spin_unlock_irqrestore(&pcc_lock, flags);

	return count;
}

static struct process_filter_opts_attribute process_filter_opts_cap_cumulative =
	__CONFIGFS_ATTR(cap_cumulative, S_IRUGO | S_IWUSR, process_filter_opts_cap_cumulative_show,
			process_filter_opts_cap_cumulative_store);

static ssize_t process_filter_opts_command_filter_show(struct process_filter_opts *opts, char *page){
	unsigned long flags;
	ssize_t len;
	spin_lock_irqsave(&pcc_lock, flags);
	len = sprintf(page, "%s\n", opts->command_filter);
	spin_unlock_irqrestore(&pcc_lock, flags);
	return len;
}

static ssize_t process_filter_opts_command_filter_store(struct process_filter_opts *opts, const char *page, size_t count){
	unsigned long flags;
	size_t proper_count;

	for(proper_count = 0; proper_count < count && page[proper_count] !='\0'; ++proper_count)
		if(page[proper_count] == '\n') {
			break;
		}
	++proper_count;
	if (proper_count > TASK_COMM_LEN+1) {
		proper_count = TASK_COMM_LEN+1;
	}
	spin_lock_irqsave(&pcc_lock, flags);
	strlcpy(opts->command_filter, page, proper_count);
	spin_unlock_irqrestore(&pcc_lock, flags);

	return count;
}

static struct process_filter_opts_attribute process_filter_opts_command_filter =
	__CONFIGFS_ATTR(command_filter, S_IRUGO | S_IWUSR, process_filter_opts_command_filter_show,
			process_filter_opts_command_filter_store);

static struct configfs_attribute *process_filter_attrs[] = {
	&process_filter_opts_command_filter.attr,
	&process_filter_opts_cap_cumulative.attr,
	NULL,
};

static struct config_item_type process_filter_type = {
	.ct_item_ops = &process_filter_item_opts,
	.ct_attrs = process_filter_attrs,
	.ct_owner = THIS_MODULE,
};


static struct process_filter_opts process_filter_subsys = {
	.subsys = {
		.su_group = {
			.cg_item = {
				.ci_namebuf = "process_filter",
				.ci_type = &process_filter_type,
			},
		},
		.su_mutex = __MUTEX_INITIALIZER(process_filter_subsys.subsys.su_mutex),
	},
};

static void capable_check_func(const struct cred *cred, int cap){
	unsigned long flags;
	int print_cap_mask = 1;
	int cap_mask = CAP_TO_MASK(cap);

	pid_t tmp_pid = get_current()->tgid;
	if(tmp_pid <= 1 || tmp_pid > PID_MAX_DEFAULT
        || cred != current_cred()){
		return;
	}

	spin_lock_irqsave(&pcc_lock, flags);
	if(process_filter_subsys.command_filter[0] != '\0'
	&& strcmp(process_filter_subsys.command_filter, get_current()->comm) != 0){
                print_cap_mask = 0;
	}
	if(process_filter_subsys.cap_cumulative == 0 && print_cap_mask){
		printk(KERN_NOTICE "Current capability set asked for comm: %s, pid: %ld is 0x%x\n",
			get_current()->comm, (long)tmp_pid, cap_mask);
		goto end_lock;
	}
	if((process_filter_subsys.pcc_array[(unsigned long)tmp_pid - 1].cap_mask | cap_mask)
	!= process_filter_subsys.pcc_array[(unsigned long)tmp_pid - 1].cap_mask){
		process_filter_subsys.pcc_array[(unsigned long)tmp_pid - 1].cap_mask
			= process_filter_subsys.pcc_array[(unsigned long)tmp_pid - 1].cap_mask | cap_mask;
		if (print_cap_mask) {
			printk(KERN_NOTICE "Collected capability set asked for comm: %s, pid: %ld is 0x%x\n",
				get_current()->comm, (long)tmp_pid,
				process_filter_subsys.pcc_array[(unsigned long)tmp_pid - 1].cap_mask);
		}
	}

end_lock:
	spin_unlock_irqrestore(&pcc_lock, flags);
}

static int jp_cap_capable_entry(const struct cred *cred,
				struct user_namespace *targ_ns,
				int cap, int audit){
	capable_check_func(cred, cap);
	jprobe_return();
	return 0;
}

static int jp_security_capable_entry(const struct cred *cred,
				struct user_namespace *targ_ns,
				int cap){
	capable_check_func(cred, cap);
	jprobe_return();
	return 0;
}

static int jp_security_capable_noaudit_entry(const struct cred *cred,
				struct user_namespace *targ_ns,
				int cap){
	capable_check_func(cred, cap);
	jprobe_return();
	return 0;
}

static void jp_wake_up_new_task_entry(struct task_struct *p){
	unsigned long flags;

	if(p->tgid <= 1 || p->tgid > PID_MAX_DEFAULT || get_current()->tgid == p->tgid){
		goto end;
	}

	spin_lock_irqsave(&pcc_lock, flags);
	process_filter_subsys.pcc_array[(unsigned long)p->tgid - 1].cap_mask = 0;
	if(process_filter_subsys.command_filter[0] == '\0'
	|| !strncmp(process_filter_subsys.command_filter, get_current()->comm, TASK_COMM_LEN)){
		printk(KERN_NOTICE "Processs created. comm: %s, pid: %ld\n",
			get_current()->comm, (long)p->tgid);
	}
	spin_unlock_irqrestore(&pcc_lock, flags);

end:
	jprobe_return();
}

static int jp_do_execve_entry(const char *filename,
			const char __user *const __user *__argv,
			const char __user *const __user *__envp) {
	unsigned long flags;
	const char* fname;

	pid_t tmp_pid = get_current()->tgid;
	if (tmp_pid <= 1 || tmp_pid > PID_MAX_DEFAULT){
		goto end;
	}

	if(strrchr(filename, '/') == NULL) {
		fname = filename;
	} else {
		fname = strrchr(filename, '/') + sizeof(char);
	}

	spin_lock_irqsave(&pcc_lock, flags);
	if(process_filter_subsys.command_filter[0] == '\0'
	|| !strncmp(process_filter_subsys.command_filter, fname, TASK_COMM_LEN)
	|| !strncmp(process_filter_subsys.command_filter, get_current()->comm, TASK_COMM_LEN)){
		printk(KERN_NOTICE "Processs execed. comm: %s, comm_execed: %s, pid: %ld\n",
			get_current()->comm, filename, (long)tmp_pid);
	}
	spin_unlock_irqrestore(&pcc_lock, flags);

end:
	jprobe_return();
	return 0;
}

static int krp_do_execve_handler(struct kretprobe_instance *ri, struct pt_regs *regs){
	unsigned long flags;

	spin_lock_irqsave(&pcc_lock, flags);
	process_filter_subsys.pcc_array[(unsigned long)get_current()->tgid - 1].cap_mask = 0;
	spin_unlock_irqrestore(&pcc_lock, flags);
	return 0;
}

static struct jprobe jp_cap_capable = {
	.entry = jp_cap_capable_entry,
	.kp = {
		.symbol_name = "cap_capable",
	},
};

static struct jprobe jp_security_capable = {
	.entry = jp_security_capable_entry,
	.kp = {
		.symbol_name = "security_capable",
	},
};

static struct jprobe jp_security_capable_noaudit = {
	.entry = jp_security_capable_noaudit_entry,
	.kp = {
		.symbol_name = "security_capable_noaudit",
	},
};

static struct jprobe jp_wake_up_new_task = {
	.entry = jp_wake_up_new_task_entry,
	.kp = {
		.symbol_name = "wake_up_new_task",
	},
};

static struct jprobe jp_do_execve = {
	.entry = jp_do_execve_entry,
	.kp = {
		.symbol_name = "do_execve",
	},
};

static struct kretprobe krp_do_execve = {
	.handler = krp_do_execve_handler,
	.kp = {
		.symbol_name = "do_execve",
	},
	.maxactive = 30,
};

static int __init cap_probe_init(void)
{
	int ret;
	unsigned long i;

	strlcpy(process_filter_subsys.command_filter, arg_command_filter, TASK_COMM_LEN+1);
	process_filter_subsys.cap_cumulative = arg_cap_cumulative;

	for(i=0; i<PID_MAX_DEFAULT; ++i){
		process_filter_subsys.pcc_array[i].cap_mask=0;
	}

	config_group_init(&process_filter_subsys.subsys.su_group);

	if ((ret = configfs_register_subsystem(&process_filter_subsys.subsys)) < 0){
		printk("%s: register_subsystem for subsystem name \"%s\" failed, returned %d\n",
			__FUNCTION__, process_filter_subsys.subsys.su_group.cg_item.ci_namebuf, ret);
		goto err;
	}

	if ((ret = register_jprobe(&jp_cap_capable)) < 0) {
		printk("%s: register_jprobe for cap_capable failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	if ((ret = register_jprobe(&jp_security_capable)) < 0) {
		printk("%s: register_jprobe for security_capable failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	if ((ret = register_jprobe(&jp_security_capable_noaudit)) < 0) {
		printk("%s: register_jprobe for security_capable_noaudit failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	if ((ret = register_jprobe(&jp_wake_up_new_task)) < 0) {
		printk("%s: register_jprobe for wake_up_new_task failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	if ((ret = register_jprobe(&jp_do_execve)) < 0) {
		printk("%s: register_jprobe for do_execve failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	if ((ret = register_kretprobe(&krp_do_execve)) < 0) {
		printk("%s: register_jprobe for do_execve failed, returned %d\n",
			__FUNCTION__, ret);
		goto err;
	}

	return 0;

err:
	unregister_kretprobe(&krp_do_execve);
	unregister_jprobe(&jp_do_execve);
	unregister_jprobe(&jp_wake_up_new_task);
	unregister_jprobe(&jp_security_capable);
	unregister_jprobe(&jp_security_capable_noaudit);
	unregister_jprobe(&jp_cap_capable);

	configfs_unregister_subsystem(&process_filter_subsys.subsys);

	return -1;
}

static void __exit cap_probe_exit(void)
{
	unregister_kretprobe(&krp_do_execve);
	unregister_jprobe(&jp_do_execve);
	unregister_jprobe(&jp_wake_up_new_task);
	unregister_jprobe(&jp_security_capable);
	unregister_jprobe(&jp_security_capable_noaudit);
	unregister_jprobe(&jp_cap_capable);

	configfs_unregister_subsystem(&process_filter_subsys.subsys);

	printk("capable kprobes unregistered\n");
}

module_init(cap_probe_init);
module_exit(cap_probe_exit);

MODULE_LICENSE("GPL");
