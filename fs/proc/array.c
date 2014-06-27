/*
 *  linux/fs/proc/array.c
 *
 *  Copyright (C) 1992  by Linus Torvalds
 *  based on ideas by Darren Senn
 *
 * Fixes:
 * Michael. K. Johnson: stat,statm extensions.
 *                      <johnsonm@stolaf.edu>
 *
 * Pauline Middelink :  Made cmdline,envline only break at '\0's, to
 *                      make sure SET_PROCTITLE works. Also removed
 *                      bad '!' which forced address recalculation for
 *                      EVERY character on the current page.
 *                      <middelin@polyware.iaf.nl>
 *
 * Danny ter Haar    :	added cpuinfo
 *			<dth@cistron.nl>
 *
 * Alessandro Rubini :  profile extension.
 *                      <rubini@ipvvis.unipv.it>
 *
 * Jeff Tranter      :  added BogoMips field to cpuinfo
 *                      <Jeff_Tranter@Mitel.COM>
 *
 * Bruno Haible      :  remove 4K limit for the maps file
 *			<haible@ma2s2.mathematik.uni-karlsruhe.de>
 *
 * Yves Arrouye      :  remove removal of trailing spaces in get_array.
 *			<Yves.Arrouye@marin.fdn.fr>
 *
 * Jerome Forissier  :  added per-CPU time information to /proc/stat
 *                      and /proc/<pid>/cpu extension
 *                      <forissier@isia.cma.fr>
 *			- Incorporation and non-SMP safe operation
 *			of forissier patch in 2.1.78 by
 *			Hans Marcus <crowbar@concepts.nl>
 *
 * aeb@cwi.nl        :  /proc/partitions
 *
 *
 * Alan Cox	     :  security fixes.
 *			<alan@lxorguk.ukuu.org.uk>
 *
 * Al Viro           :  safe handling of mm_struct
 *
 * Gerhard Wichert   :  added BIGMEM support
 * Siemens AG           <Gerhard.Wichert@pdb.siemens.de>
 *
 * Al Viro & Jeff Garzik :  moved most of the thing into base.c and
 *			 :  proc_misc.c. The rest may eventually go into
 *			 :  base.c too.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/pagemap.h>
#include <linux/swap.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/highmem.h>
#include <linux/file.h>
#include <linux/times.h>
#include <linux/cpuset.h>
#include <linux/rcupdate.h>
#include <linux/delayacct.h>
#include <linux/ptrace.h>
#include <linux/tracehook.h>
#include <linux/proc_info.h>

#include <asm/pgtable.h>
#include <asm/processor.h>
#include "internal.h"

int proc_pid_status(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task)
{
	struct mm_struct *mm = get_task_mm(task);

	proc_pid_status_mm(m, ns, pid, task, mm);
	if (mm)
		mmput(mm);

	return 0;
}

static int do_task_stat(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task, int whole)
{
	struct mm_struct *mm = get_task_mm(task);

	task_stat_mm(m, ns, pid, task, whole, mm);
	if (mm)
		mmput(mm);

	return 0;
}

int proc_tid_stat(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task)
{
	return do_task_stat(m, ns, pid, task, 0);
}

int proc_tgid_stat(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task)
{
	return do_task_stat(m, ns, pid, task, 1);
}

int proc_pid_statm(struct seq_file *m, struct pid_namespace *ns,
			struct pid *pid, struct task_struct *task)
{
	struct mm_struct *mm = get_task_mm(task);

	proc_pid_statm_mm(m, ns, pid, task, mm);
	if (mm)
		mmput(mm);

	return 0;
}

#ifdef CONFIG_CHECKPOINT_RESTORE
static struct pid *
get_children_pid(struct inode *inode, struct pid *pid_prev, loff_t pos)
{
	struct task_struct *start, *task;
	struct pid *pid = NULL;

	read_lock(&tasklist_lock);

	start = pid_task(proc_pid(inode), PIDTYPE_PID);
	if (!start)
		goto out;

	/*
	 * Lets try to continue searching first, this gives
	 * us significant speedup on children-rich processes.
	 */
	if (pid_prev) {
		task = pid_task(pid_prev, PIDTYPE_PID);
		if (task && task->real_parent == start &&
		    !(list_empty(&task->sibling))) {
			if (list_is_last(&task->sibling, &start->children))
				goto out;
			task = list_first_entry(&task->sibling,
						struct task_struct, sibling);
			pid = get_pid(task_pid(task));
			goto out;
		}
	}

	/*
	 * Slow search case.
	 *
	 * We might miss some children here if children
	 * are exited while we were not holding the lock,
	 * but it was never promised to be accurate that
	 * much.
	 *
	 * "Just suppose that the parent sleeps, but N children
	 *  exit after we printed their tids. Now the slow paths
	 *  skips N extra children, we miss N tasks." (c)
	 *
	 * So one need to stop or freeze the leader and all
	 * its children to get a precise result.
	 */
	list_for_each_entry(task, &start->children, sibling) {
		if (pos-- == 0) {
			pid = get_pid(task_pid(task));
			break;
		}
	}

out:
	read_unlock(&tasklist_lock);
	return pid;
}

static int children_seq_show(struct seq_file *seq, void *v)
{
	struct inode *inode = seq->private;
	pid_t pid;

	pid = pid_nr_ns(v, inode->i_sb->s_fs_info);
	return seq_printf(seq, "%d ", pid);
}

static void *children_seq_start(struct seq_file *seq, loff_t *pos)
{
	return get_children_pid(seq->private, NULL, *pos);
}

static void *children_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct pid *pid;

	pid = get_children_pid(seq->private, v, *pos + 1);
	put_pid(v);

	++*pos;
	return pid;
}

static void children_seq_stop(struct seq_file *seq, void *v)
{
	put_pid(v);
}

static const struct seq_operations children_seq_ops = {
	.start	= children_seq_start,
	.next	= children_seq_next,
	.stop	= children_seq_stop,
	.show	= children_seq_show,
};

static int children_seq_open(struct inode *inode, struct file *file)
{
	struct seq_file *m;
	int ret;

	ret = seq_open(file, &children_seq_ops);
	if (ret)
		return ret;

	m = file->private_data;
	m->private = inode;

	return ret;
}

int children_seq_release(struct inode *inode, struct file *file)
{
	seq_release(inode, file);
	return 0;
}

const struct file_operations proc_tid_children_operations = {
	.open    = children_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = children_seq_release,
};
#endif /* CONFIG_CHECKPOINT_RESTORE */
