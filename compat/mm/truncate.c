#include <linux/kernel.h>
#include <linux/backing-dev.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/export.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/pagevec.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/buffer_head.h>	/* grr. try_to_release_page,
				   do_invalidatepage */
#include <linux/cleancache.h>
#include <linux/rmap.h>

#include <linux/fs.h>
#include <linux/mm.h>

/**
 * truncate_inode_pages_final - truncate *all* pages before inode dies
 * @mapping: mapping to truncate
 *
 * Called under (and serialized by) inode->i_mutex.
 *
 * Filesystems have to use this in the .evict_inode path to inform the
 * VM that this is the final truncate and the inode is going away.
 */
void truncate_inode_pages_final(struct address_space *mapping)
{
	unsigned long nrpages;

	/*
	 * Page reclaim can not participate in regular inode lifetime
	 * management (can't call iput()) and thus can race with the
	 * inode teardown.  Tell it when the address space is exiting,
	 * so that it does not install eviction information after the
	 * final truncate has begun.
	 */
	mapping_set_exiting(mapping);

	/*
	 * When reclaim installs eviction entries, it increases
	 * nrshadows first, then decreases nrpages.  Make sure we see
	 * this in the right order or we might miss an entry.
	 */
	nrpages = mapping->nrpages;
	smp_rmb();

	if (nrpages) {
		/*
		 * As truncation uses a lockless tree lookup, cycle
		 * the tree lock to make sure any ongoing tree
		 * modification that does not see AS_EXITING is
		 * completed before starting the final truncate.
		 */
		spin_lock_irq(&mapping->tree_lock);
		spin_unlock_irq(&mapping->tree_lock);

		truncate_inode_pages(mapping, 0);
	}
}
EXPORT_SYMBOL(truncate_inode_pages_final);
