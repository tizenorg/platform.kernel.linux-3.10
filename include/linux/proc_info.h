#ifndef _LINUX_PROC_INFO_H
#define _LINUX_PROC_INFO_H

#include <linux/types.h>
#include <linux/fdtable.h>
#include <linux/seq_file.h>
#include <linux/user_namespace.h>
#include <linux/ptrace.h>

void task_mem(struct seq_file *m, struct mm_struct *mm);
unsigned long task_vsize(struct mm_struct *mm);
unsigned long task_statm(struct mm_struct *mm,
			 unsigned long *shared, unsigned long *text,
			 unsigned long *data, unsigned long *resident);
void task_name(struct seq_file *m, struct task_struct *p);
const char *get_task_state(struct task_struct *tsk);
void task_state(struct seq_file *m, struct pid_namespace *ns,
		struct pid *pid, struct task_struct *p);
void task_sig(struct seq_file *m, struct task_struct *p);
void task_cap(struct seq_file *m, struct task_struct *p);
void task_seccomp(struct seq_file *m, struct task_struct *p);
void task_context_switch_counts(struct seq_file *m, struct task_struct *p);
void task_cpus_allowed(struct seq_file *m, struct task_struct *task);
void render_sigset_t(struct seq_file *m, const char *header, sigset_t *set);
int proc_pid_status_mm(struct seq_file *m, struct pid_namespace *ns,
		       struct pid *pid, struct task_struct *task,
		       struct mm_struct *mm);
int task_stat_mm(struct seq_file *m, struct pid_namespace *ns,
		 struct pid *pid, struct task_struct *task, int whole,
		 struct mm_struct *mm);
int proc_pid_statm_mm(struct seq_file *m, struct pid_namespace *ns,
		      struct pid *pid, struct task_struct *task,
		      struct mm_struct *mm);

#endif /* _LINUX_PROC_INFO_H */
