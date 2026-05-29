// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal MHI pipe for SDX55M image loading and EFS sync.
 *
 * This intentionally mirrors the Android downstream mhi_uci style userspace
 * contract more closely than the WWAN port path: a single process owns the
 * SAHARA state machine through /dev/mhi_pipe_2 and the post-AMSS EFS sync
 * session through /dev/mhi_pipe_10.
 */
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-direction.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define MHI_SAHARA_RX_MTU		0x8000
#define MHI_SAHARA_TX_CHUNK		MHI_SAHARA_RX_MTU
#define MHI_SAHARA_TX_MTU		0x100000
#define MHI_SAHARA_EVENT_POLL_MS	20
#define MHI_SAHARA_TX_POLL_SLEEP_US	1000
#define SAHARA_HELLO_CMD		1
#define SAHARA_HELLO_RESP_CMD		2
#define SAHARA_READ_DATA_CMD		3
#define SAHARA_END_IMAGE_CMD		4
#define SAHARA_DONE_CMD		5
#define SAHARA_DONE_RESP_CMD		6
#define SAHARA_MEMORY_READ_CMD		10
#define SAHARA_SWITCH_MODE_CMD		12
#define SAHARA_CMD_EXEC_RESP_CMD	14
#define SAHARA_MEMORY_READ64_CMD	17
#define SAHARA_MODE_COMMAND		3
#define SAHARA_HELLO_RESP_LEN		48
#define SAHARA_DONE_LEN		8

static unsigned int mhi_sahara_restart_after_ul_ms;
module_param_named(restart_after_ul_ms,
		   mhi_sahara_restart_after_ul_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_ul_ms,
		 "Restart SDX55M SBL SAHARA channels after successful UL completion, disabled by default");

static unsigned int mhi_sahara_restart_after_hello_resp_ms;
module_param_named(restart_after_hello_resp_ms,
		   mhi_sahara_restart_after_hello_resp_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_hello_resp_ms,
		 "Override diagnostic restart delay after SAHARA HELLO_RESP UL completions");

static unsigned int mhi_sahara_restart_after_first_hello_resp_ms;
module_param_named(restart_after_first_hello_resp_ms,
		   mhi_sahara_restart_after_first_hello_resp_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_first_hello_resp_ms,
		 "Override diagnostic restart delay after the first SAHARA HELLO_RESP UL completion in an open");

static unsigned int mhi_sahara_restart_after_next_hello_resp_ms;
module_param_named(restart_after_next_hello_resp_ms,
		   mhi_sahara_restart_after_next_hello_resp_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_next_hello_resp_ms,
		 "Override diagnostic restart delay after later SAHARA HELLO_RESP UL completions in an open");

static unsigned int mhi_sahara_restart_after_command_hello_resp_ms;
module_param_named(restart_after_command_hello_resp_ms,
		   mhi_sahara_restart_after_command_hello_resp_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_command_hello_resp_ms,
		 "Override diagnostic restart delay after SAHARA HELLO_RESP mode=3 command-mode completions");

static unsigned int mhi_sahara_restart_after_done_ms;
module_param_named(restart_after_done_ms,
		   mhi_sahara_restart_after_done_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_done_ms,
		 "Override diagnostic restart delay after SAHARA DONE UL completions");

static unsigned int mhi_sahara_restart_after_switch_ms;
module_param_named(restart_after_switch_ms,
		   mhi_sahara_restart_after_switch_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_switch_ms,
		 "Override diagnostic restart delay after SAHARA CMD_SWITCH_MODE UL completions");

static unsigned int mhi_sahara_restart_after_data_ms;
module_param_named(restart_after_data_ms,
		   mhi_sahara_restart_after_data_ms, uint, 0644);
MODULE_PARM_DESC(restart_after_data_ms,
		 "Override diagnostic restart delay after SAHARA image data UL completions");

static bool mhi_sahara_restart_resync_db;
module_param_named(restart_resync_db, mhi_sahara_restart_resync_db, bool, 0644);
MODULE_PARM_DESC(restart_resync_db,
		 "Resync SDX55M SBL SAHARA cached doorbells after diagnostic restart");

static bool mhi_sahara_restart_periodic;
module_param_named(restart_periodic, mhi_sahara_restart_periodic, bool, 0644);
MODULE_PARM_DESC(restart_periodic,
		 "Keep repeating SDX55M SBL SAHARA full channel restarts while idle, disabled by default");

static bool mhi_sahara_restart_control_only;
module_param_named(restart_control_only, mhi_sahara_restart_control_only, bool, 0644);
MODULE_PARM_DESC(restart_control_only,
		 "Only schedule global diagnostic restarts after SAHARA control packets");

static bool mhi_sahara_restart_idle_only;
module_param_named(restart_idle_only, mhi_sahara_restart_idle_only, bool, 0644);
MODULE_PARM_DESC(restart_idle_only,
		 "Only schedule diagnostic restarts if no valid SAHARA DL packet arrived after the UL was queued");

static bool mhi_sahara_cancel_restart_on_hello = true;
module_param_named(cancel_restart_on_hello,
		   mhi_sahara_cancel_restart_on_hello, bool, 0644);
MODULE_PARM_DESC(cancel_restart_on_hello,
		 "Cancel pending diagnostic restarts when a SAHARA HELLO DL packet arrives");

static bool mhi_sahara_cancel_restart_on_done_resp = true;
module_param_named(cancel_restart_on_done_resp,
		   mhi_sahara_cancel_restart_on_done_resp, bool, 0644);
MODULE_PARM_DESC(cancel_restart_on_done_resp,
		 "Cancel pending diagnostic restarts when a SAHARA DONE_RESP DL packet arrives");

static bool mhi_sahara_unprepare_on_close;
module_param_named(unprepare_on_close, mhi_sahara_unprepare_on_close, bool, 0644);
MODULE_PARM_DESC(unprepare_on_close,
		 "Unprepare SAHARA channels on last close instead of keeping the Android-like pipe lifecycle");

static unsigned int mhi_sahara_post_write_poll_ms = 50;
module_param_named(post_write_poll_ms,
		   mhi_sahara_post_write_poll_ms, uint, 0644);
MODULE_PARM_DESC(post_write_poll_ms,
		 "Poll SDX55M SBL event rings after each userspace write until queued TX completions are observed");

enum mhi_sahara_flags {
	MHI_SAHARA_OPEN,
	MHI_SAHARA_PREPARED,
	MHI_SAHARA_RX_REFILL,
	MHI_SAHARA_REMOVING,
};

struct mhi_sahara_buf {
	struct list_head node;
	size_t len;
	size_t pos;
	bool restart_after_completion;
	unsigned int restart_delay_ms;
	u32 cmd;
	unsigned int rx_seq;
	u8 data[];
};

struct mhi_sahara_dev {
	struct mhi_device *mhi_dev;
	struct miscdevice miscdev;
	bool is_sahara;

	unsigned long flags;
	/* Protects open, prepare, restart, and remove state transitions. */
	struct mutex state_lock;
	/* Serializes userspace reads against ready-queue purges. */
	struct mutex read_lock;
	/* Serializes userspace writes and MHI TX queue-full waiting. */
	struct mutex tx_lock;

	/* Protects rx_ready, rx_ready_count, and rx_budget. */
	spinlock_t rx_lock;
	struct list_head rx_ready;
	unsigned int rx_ready_count;
	unsigned int rx_budget;
	size_t raw_rx_remaining;
	atomic_t rx_seq;
	atomic_t tx_seq;
	unsigned int hello_resp_count;
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	struct work_struct rx_refill;
	struct delayed_work event_poll;
	struct delayed_work sahara_restart;
};

static struct mhi_sahara_dev *mhi_sahara_dump_dev;
static DEFINE_MUTEX(mhi_sahara_dump_lock);

static void mhi_sahara_refill_work(struct work_struct *work);

static bool mhi_sahara_tx_done(struct mhi_sahara_dev *sahara,
			       unsigned int target_seq)
{
	return (int)(atomic_read(&sahara->tx_seq) - target_seq) >= 0;
}

static struct mhi_sahara_buf *mhi_sahara_buf_from_data(void *data)
{
	return (struct mhi_sahara_buf *)((u8 *)data -
					offsetof(struct mhi_sahara_buf, data));
}

static const char *mhi_sdx55m_pipe_name(struct mhi_device *mhi_dev)
{
	if (!mhi_dev->name)
		return NULL;
	if (!strcmp(mhi_dev->name, "SAHARA"))
		return "mhi_pipe_2";
	if (!strcmp(mhi_dev->name, "EFS"))
		return "mhi_pipe_10";

	return NULL;
}

static bool mhi_sahara_is_sdx55m(struct mhi_sahara_dev *sahara)
{
	struct mhi_controller *cntrl = sahara->mhi_dev->mhi_cntrl;

	return cntrl->name && !strcmp(cntrl->name, "qcom-sdx55m") &&
	       mhi_sdx55m_pipe_name(sahara->mhi_dev);
}

static bool mhi_sahara_rx_valid(const struct mhi_sahara_buf *buf, size_t len)
{
	u32 cmd, pkt_len;

	if (len < 8)
		return false;

	cmd = get_unaligned_le32(buf->data);
	pkt_len = get_unaligned_le32(buf->data + 4);

	return cmd && pkt_len >= 8 && pkt_len <= len;
}

enum mhi_sahara_tx_kind {
	MHI_SAHARA_TX_DATA,
	MHI_SAHARA_TX_HELLO_RESP,
	MHI_SAHARA_TX_DONE,
	MHI_SAHARA_TX_OTHER_CONTROL,
};

static enum mhi_sahara_tx_kind
mhi_sahara_tx_kind(const struct mhi_sahara_buf *buf, size_t len, u32 *cmd)
{
	u32 pkt_len;

	*cmd = 0;
	if (len < 8)
		return MHI_SAHARA_TX_DATA;

	*cmd = get_unaligned_le32(buf->data);
	pkt_len = get_unaligned_le32(buf->data + 4);

	if (*cmd == SAHARA_HELLO_RESP_CMD && len == SAHARA_HELLO_RESP_LEN &&
	    pkt_len == SAHARA_HELLO_RESP_LEN)
		return MHI_SAHARA_TX_HELLO_RESP;

	if (*cmd == SAHARA_DONE_CMD && len == SAHARA_DONE_LEN &&
	    pkt_len == SAHARA_DONE_LEN)
		return MHI_SAHARA_TX_DONE;

	if (*cmd && pkt_len >= 8 && pkt_len == len)
		return MHI_SAHARA_TX_OTHER_CONTROL;

	return MHI_SAHARA_TX_DATA;
}

static size_t
mhi_sahara_tx_raw_rx_len(const struct mhi_sahara_buf *buf, size_t len)
{
	u64 raw_len;
	u32 cmd, pkt_len;

	if (len < 8)
		return 0;

	cmd = get_unaligned_le32(buf->data);
	pkt_len = get_unaligned_le32(buf->data + 4);

	if (cmd == SAHARA_MEMORY_READ_CMD && len == 16 && pkt_len == 16)
		return get_unaligned_le32(buf->data + 12);

	if (cmd != SAHARA_MEMORY_READ64_CMD || len != 24 || pkt_len != 24)
		return 0;

	raw_len = get_unaligned_le64(buf->data + 16);
	if (raw_len > SIZE_MAX)
		return 0;

	return raw_len;
}

static unsigned int
mhi_sahara_hello_restart_delay(struct mhi_sahara_dev *sahara,
			       const struct mhi_sahara_buf *buf)
{
	u32 mode;

	sahara->hello_resp_count++;

	mode = get_unaligned_le32(buf->data + 20);
	if (mode == SAHARA_MODE_COMMAND &&
	    mhi_sahara_restart_after_command_hello_resp_ms)
		return mhi_sahara_restart_after_command_hello_resp_ms;

	if (sahara->hello_resp_count == 1 &&
	    mhi_sahara_restart_after_first_hello_resp_ms)
		return mhi_sahara_restart_after_first_hello_resp_ms;

	if (sahara->hello_resp_count > 1 &&
	    mhi_sahara_restart_after_next_hello_resp_ms)
		return mhi_sahara_restart_after_next_hello_resp_ms;

	if (mhi_sahara_restart_after_hello_resp_ms)
		return mhi_sahara_restart_after_hello_resp_ms;

	return mhi_sahara_restart_after_ul_ms;
}

static unsigned int
mhi_sahara_tx_restart_delay(struct mhi_sahara_dev *sahara,
			    const struct mhi_sahara_buf *buf, size_t len, u32 *cmd)
{
	bool typed_restart;

	if (!sahara->is_sahara)
		return 0;

	switch (mhi_sahara_tx_kind(buf, len, cmd)) {
	case MHI_SAHARA_TX_HELLO_RESP:
		return mhi_sahara_hello_restart_delay(sahara, buf);
	case MHI_SAHARA_TX_DONE:
		if (mhi_sahara_restart_after_done_ms)
			return mhi_sahara_restart_after_done_ms;
		return mhi_sahara_restart_after_ul_ms;
	case MHI_SAHARA_TX_DATA:
		if (mhi_sahara_restart_after_data_ms)
			return mhi_sahara_restart_after_data_ms;
		if (mhi_sahara_restart_control_only)
			return 0;
		return mhi_sahara_restart_after_ul_ms;
	case MHI_SAHARA_TX_OTHER_CONTROL:
		if (*cmd == SAHARA_SWITCH_MODE_CMD &&
		    mhi_sahara_restart_after_switch_ms)
			return mhi_sahara_restart_after_switch_ms;
		typed_restart = mhi_sahara_restart_after_hello_resp_ms ||
				mhi_sahara_restart_after_first_hello_resp_ms ||
				mhi_sahara_restart_after_next_hello_resp_ms ||
				mhi_sahara_restart_after_command_hello_resp_ms ||
				mhi_sahara_restart_after_switch_ms ||
				mhi_sahara_restart_after_done_ms ||
				mhi_sahara_restart_after_data_ms;
		if (typed_restart || mhi_sahara_restart_control_only)
			return 0;
		return mhi_sahara_restart_after_ul_ms;
	}

	return 0;
}

static void mhi_sahara_purge_ready(struct mhi_sahara_dev *sahara)
{
	struct mhi_sahara_buf *buf, *tmp;
	unsigned long flags;

	mutex_lock(&sahara->read_lock);
	spin_lock_irqsave(&sahara->rx_lock, flags);
	list_for_each_entry_safe(buf, tmp, &sahara->rx_ready, node) {
		list_del(&buf->node);
		kfree(buf);
	}
	sahara->rx_ready_count = 0;
	spin_unlock_irqrestore(&sahara->rx_lock, flags);
	mutex_unlock(&sahara->read_lock);
}

static bool mhi_sahara_has_ready(struct mhi_sahara_dev *sahara)
{
	unsigned long flags;
	bool ready;

	spin_lock_irqsave(&sahara->rx_lock, flags);
	ready = !list_empty(&sahara->rx_ready);
	spin_unlock_irqrestore(&sahara->rx_lock, flags);

	return ready;
}

static bool mhi_sahara_read_wake(struct mhi_sahara_dev *sahara)
{
	return mhi_sahara_has_ready(sahara) ||
	       !test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	       test_bit(MHI_SAHARA_REMOVING, &sahara->flags);
}

static bool mhi_sahara_write_wake(struct mhi_sahara_dev *sahara)
{
	return !mhi_queue_is_full(sahara->mhi_dev, DMA_TO_DEVICE) ||
	       !test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	       test_bit(MHI_SAHARA_REMOVING, &sahara->flags);
}

static void mhi_sahara_rx_budget_inc(struct mhi_sahara_dev *sahara)
{
	unsigned long flags;

	spin_lock_irqsave(&sahara->rx_lock, flags);
	sahara->rx_budget++;
	if (test_bit(MHI_SAHARA_RX_REFILL, &sahara->flags))
		schedule_work(&sahara->rx_refill);
	spin_unlock_irqrestore(&sahara->rx_lock, flags);
}

static bool mhi_sahara_rx_budget_dec(struct mhi_sahara_dev *sahara)
{
	unsigned long flags;
	bool ret = false;

	spin_lock_irqsave(&sahara->rx_lock, flags);
	if (sahara->rx_budget && test_bit(MHI_SAHARA_RX_REFILL, &sahara->flags)) {
		sahara->rx_budget--;
		ret = true;
	}
	spin_unlock_irqrestore(&sahara->rx_lock, flags);

	return ret;
}

static void mhi_sahara_event_poll_work(struct work_struct *work)
{
	struct mhi_sahara_dev *sahara;

	sahara = container_of(to_delayed_work(work), struct mhi_sahara_dev,
			      event_poll);

	if (!test_bit(MHI_SAHARA_RX_REFILL, &sahara->flags))
		return;

	mhi_sdx55m_drain_events(sahara->mhi_dev);
	schedule_delayed_work(&sahara->event_poll,
			      msecs_to_jiffies(MHI_SAHARA_EVENT_POLL_MS));
}

static void mhi_sahara_poll_tx_completions(struct mhi_sahara_dev *sahara,
					   unsigned int target_seq)
{
	struct mhi_device *mhi_dev = sahara->mhi_dev;
	unsigned long deadline;

	if (!mhi_sahara_post_write_poll_ms)
		return;

	deadline = jiffies + msecs_to_jiffies(mhi_sahara_post_write_poll_ms);
	do {
		mhi_sdx55m_drain_events(mhi_dev);
		if (mhi_sahara_tx_done(sahara, target_seq))
			return;
		if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
		    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
			return;
		usleep_range(MHI_SAHARA_TX_POLL_SLEEP_US,
			     MHI_SAHARA_TX_POLL_SLEEP_US * 2);
	} while (time_before(jiffies, deadline));

	mhi_sdx55m_drain_events(mhi_dev);
	if (!mhi_sahara_tx_done(sahara, target_seq))
		dev_warn_ratelimited(&mhi_dev->dev,
				     "Timed out polling SDX55M SAHARA TX completions seq=%d target=%u\n",
				     atomic_read(&sahara->tx_seq), target_seq);
}

static void mhi_sahara_refill_work(struct work_struct *work)
{
	struct mhi_sahara_dev *sahara;
	struct mhi_device *mhi_dev;

	sahara = container_of(work, struct mhi_sahara_dev, rx_refill);
	mhi_dev = sahara->mhi_dev;

	while (mhi_sahara_rx_budget_dec(sahara)) {
		struct mhi_sahara_buf *buf;
		int ret;

		buf = kzalloc(struct_size(buf, data, MHI_SAHARA_RX_MTU),
			      GFP_KERNEL);
		if (!buf) {
			mhi_sahara_rx_budget_inc(sahara);
			break;
		}

		ret = mhi_queue_buf(mhi_dev, DMA_FROM_DEVICE, buf->data,
				    MHI_SAHARA_RX_MTU, MHI_EOT);
		if (ret) {
			dev_err(&mhi_dev->dev, "Failed to queue SAHARA RX buffer: %d\n",
				ret);
			kfree(buf);
			mhi_sahara_rx_budget_inc(sahara);
			break;
		}

		mhi_sdx55m_drain_events(mhi_dev);
	}
}

static int mhi_sahara_prepare_locked(struct mhi_sahara_dev *sahara)
{
	int ret;

	if (test_bit(MHI_SAHARA_PREPARED, &sahara->flags))
		return 0;

	ret = mhi_prepare_for_transfer(sahara->mhi_dev);
	if (ret)
		return ret;

	set_bit(MHI_SAHARA_PREPARED, &sahara->flags);
	return 0;
}

static void mhi_sahara_start_rx(struct mhi_sahara_dev *sahara)
{
	unsigned long flags;

	spin_lock_irqsave(&sahara->rx_lock, flags);
	sahara->rx_budget = mhi_get_free_desc_count(sahara->mhi_dev,
						    DMA_FROM_DEVICE);
	set_bit(MHI_SAHARA_RX_REFILL, &sahara->flags);
	spin_unlock_irqrestore(&sahara->rx_lock, flags);

	mhi_sahara_refill_work(&sahara->rx_refill);
	schedule_delayed_work(&sahara->event_poll,
			      msecs_to_jiffies(MHI_SAHARA_EVENT_POLL_MS));
}

static void mhi_sahara_stop_rx(struct mhi_sahara_dev *sahara)
{
	clear_bit(MHI_SAHARA_RX_REFILL, &sahara->flags);
	cancel_delayed_work_sync(&sahara->sahara_restart);
	cancel_delayed_work_sync(&sahara->event_poll);
	cancel_work_sync(&sahara->rx_refill);
}

static void mhi_sahara_restart_work(struct work_struct *work)
{
	struct mhi_sahara_dev *sahara;
	struct mhi_device *mhi_dev;
	int ret;

	sahara = container_of(to_delayed_work(work), struct mhi_sahara_dev,
			      sahara_restart);
	mhi_dev = sahara->mhi_dev;

	if (!sahara->is_sahara)
		return;

	if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	    !test_bit(MHI_SAHARA_RX_REFILL, &sahara->flags) ||
	    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
		return;

	mutex_lock(&sahara->state_lock);
	if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
		goto out_unlock;

	clear_bit(MHI_SAHARA_RX_REFILL, &sahara->flags);
	cancel_delayed_work_sync(&sahara->event_poll);
	cancel_work_sync(&sahara->rx_refill);

	dev_info(&mhi_dev->dev,
		 "Restarting SDX55M SAHARA pipe channels after UL completion\n");
	mhi_unprepare_from_transfer(mhi_dev);
	clear_bit(MHI_SAHARA_PREPARED, &sahara->flags);

	ret = mhi_sahara_prepare_locked(sahara);
	if (ret) {
		dev_err(&mhi_dev->dev,
			"Failed to restart SDX55M SAHARA pipe channels: %d\n",
			ret);
		goto out_unlock;
	}

	if (mhi_sahara_restart_resync_db)
		mhi_sdx55m_resync_sahara_doorbells(mhi_dev);

	sahara->raw_rx_remaining = 0;
	mhi_sahara_start_rx(sahara);

		if (mhi_sahara_restart_periodic &&
		    mhi_sahara_restart_after_ul_ms) {
			dev_info(&mhi_dev->dev,
				 "Re-scheduling periodic SDX55M SAHARA pipe restart delay_ms=%u\n",
				 mhi_sahara_restart_after_ul_ms);
		mod_delayed_work(system_wq, &sahara->sahara_restart,
				 msecs_to_jiffies(mhi_sahara_restart_after_ul_ms));
	}

out_unlock:
	mutex_unlock(&sahara->state_lock);
}

static int mhi_sahara_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct mhi_sahara_dev *sahara;
	int ret;

	sahara = container_of(miscdev, struct mhi_sahara_dev, miscdev);

	mutex_lock(&sahara->state_lock);
	if (test_bit(MHI_SAHARA_REMOVING, &sahara->flags)) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (test_and_set_bit(MHI_SAHARA_OPEN, &sahara->flags)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	mhi_sahara_purge_ready(sahara);

	ret = mhi_sahara_prepare_locked(sahara);
	if (ret) {
		clear_bit(MHI_SAHARA_OPEN, &sahara->flags);
		goto out_unlock;
	}

	sahara->raw_rx_remaining = 0;
	sahara->hello_resp_count = 0;
	file->private_data = sahara;
	mhi_sahara_start_rx(sahara);
	ret = 0;

out_unlock:
	mutex_unlock(&sahara->state_lock);
	return ret;
}

static int mhi_sahara_release(struct inode *inode, struct file *file)
{
	struct mhi_sahara_dev *sahara = file->private_data;

	clear_bit(MHI_SAHARA_OPEN, &sahara->flags);
	mhi_sahara_stop_rx(sahara);
	wake_up_interruptible(&sahara->read_wq);
	wake_up_interruptible(&sahara->write_wq);

	mutex_lock(&sahara->state_lock);
	mhi_sahara_purge_ready(sahara);
	if (mhi_sahara_unprepare_on_close &&
	    test_bit(MHI_SAHARA_PREPARED, &sahara->flags)) {
		mhi_unprepare_from_transfer(sahara->mhi_dev);
		clear_bit(MHI_SAHARA_PREPARED, &sahara->flags);
	}
	mutex_unlock(&sahara->state_lock);

	return 0;
}

static ssize_t mhi_sahara_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct mhi_sahara_dev *sahara = file->private_data;
	struct mhi_sahara_buf *rxbuf;
	unsigned long flags;
	size_t copied, avail;
	int ret;

	if (!count)
		return 0;

	for (;;) {
		if (mutex_lock_interruptible(&sahara->read_lock))
			return -ERESTARTSYS;

		spin_lock_irqsave(&sahara->rx_lock, flags);
		if (!list_empty(&sahara->rx_ready)) {
			rxbuf = list_first_entry(&sahara->rx_ready,
						 struct mhi_sahara_buf, node);
			spin_unlock_irqrestore(&sahara->rx_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&sahara->rx_lock, flags);
		mutex_unlock(&sahara->read_lock);

		if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
		    test_bit(MHI_SAHARA_REMOVING, &sahara->flags)) {
			return -EIO;
		}

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(sahara->read_wq,
					       mhi_sahara_read_wake(sahara));
		if (ret)
			return ret;
	}

	avail = rxbuf->len - rxbuf->pos;
	copied = min(count, avail);
	if (copy_to_user(buf, rxbuf->data + rxbuf->pos, copied)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	rxbuf->pos += copied;
	if (rxbuf->pos == rxbuf->len) {
		spin_lock_irqsave(&sahara->rx_lock, flags);
		list_del(&rxbuf->node);
		sahara->rx_ready_count--;
		spin_unlock_irqrestore(&sahara->rx_lock, flags);
		kfree(rxbuf);
	}

	ret = copied;

out_unlock:
	mutex_unlock(&sahara->read_lock);
	return ret;
}

static ssize_t mhi_sahara_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct mhi_sahara_dev *sahara = file->private_data;
	unsigned int restart_delay_ms = 0;
	u32 restart_cmd = 0;
	unsigned int tx_start_seq;
	unsigned int tx_queued = 0;
	size_t queued = 0;
	int ret;

	if (!count)
		return 0;
	if (count > MHI_SAHARA_TX_MTU)
		return -EMSGSIZE;
	if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
		return -EIO;

	if (mutex_lock_interruptible(&sahara->tx_lock))
		return -ERESTARTSYS;

	tx_start_seq = atomic_read(&sahara->tx_seq);

	while (queued < count) {
		struct mhi_sahara_buf *txbuf;
		enum mhi_flags flags;
		size_t raw_rx_len = 0;
		size_t chunk;
		bool last;

		while (mhi_queue_is_full(sahara->mhi_dev, DMA_TO_DEVICE)) {
			if (file->f_flags & O_NONBLOCK) {
				ret = queued ? queued : -EAGAIN;
				goto out_unlock;
			}

			ret = wait_event_interruptible(sahara->write_wq,
						       mhi_sahara_write_wake(sahara));
			if (ret) {
				if (queued)
					ret = queued;
				goto out_unlock;
			}
			if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
			    test_bit(MHI_SAHARA_REMOVING, &sahara->flags)) {
				ret = queued ? queued : -EIO;
				goto out_unlock;
			}
		}

		chunk = min_t(size_t, count - queued, MHI_SAHARA_TX_CHUNK);
		last = queued + chunk == count;

		txbuf = kzalloc(struct_size(txbuf, data, chunk), GFP_KERNEL);
		if (!txbuf) {
			ret = queued ? queued : -ENOMEM;
			goto out_unlock;
		}
		txbuf->len = chunk;
		txbuf->rx_seq = atomic_read(&sahara->rx_seq);

		if (copy_from_user(txbuf->data, buf + queued, chunk)) {
			kfree(txbuf);
			ret = queued ? queued : -EFAULT;
			goto out_unlock;
		}

		if (!queued) {
			raw_rx_len = mhi_sahara_tx_raw_rx_len(txbuf, chunk);
			if (raw_rx_len) {
				sahara->raw_rx_remaining = raw_rx_len;
				dev_dbg(&sahara->mhi_dev->dev,
					"Expecting SAHARA raw memory DL bytes=%zu\n",
					raw_rx_len);
			}
			restart_delay_ms =
				mhi_sahara_tx_restart_delay(sahara, txbuf,
							    chunk,
							    &restart_cmd);
		}
		if (last) {
			txbuf->cmd = restart_cmd;
			txbuf->restart_delay_ms = restart_delay_ms;
			txbuf->restart_after_completion = restart_delay_ms != 0;
		}

		flags = last ? MHI_EOT : MHI_CHAIN;
		ret = mhi_queue_buf(sahara->mhi_dev, DMA_TO_DEVICE,
				    txbuf->data, txbuf->len, flags);
		if (ret) {
			if (raw_rx_len)
				sahara->raw_rx_remaining = 0;
			kfree(txbuf);
			ret = queued ? queued : ret;
			goto out_unlock;
		}

		queued += chunk;
		tx_queued++;
		mhi_sdx55m_drain_events(sahara->mhi_dev);
	}

	mhi_sahara_poll_tx_completions(sahara, tx_start_seq + tx_queued);
	ret = count;

out_unlock:
	mutex_unlock(&sahara->tx_lock);
	return ret;
}

static __poll_t mhi_sahara_poll(struct file *file, poll_table *wait)
{
	struct mhi_sahara_dev *sahara = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &sahara->read_wq, wait);
	poll_wait(file, &sahara->write_wq, wait);

	if (mhi_sahara_has_ready(sahara))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (test_bit(MHI_SAHARA_OPEN, &sahara->flags) &&
	    !test_bit(MHI_SAHARA_REMOVING, &sahara->flags) &&
	    !mhi_queue_is_full(sahara->mhi_dev, DMA_TO_DEVICE))
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
		mask |= EPOLLHUP;

	return mask;
}

static const struct file_operations mhi_sahara_fops = {
	.owner = THIS_MODULE,
	.open = mhi_sahara_open,
	.release = mhi_sahara_release,
	.read = mhi_sahara_read,
	.write = mhi_sahara_write,
	.poll = mhi_sahara_poll,
	.llseek = noop_llseek,
};

static int mhi_sahara_dump_set(const char *val, const struct kernel_param *kp)
{
	struct mhi_sahara_dev *sahara;
	bool trigger;
	int ret;

	ret = kstrtobool(val, &trigger);
	if (ret)
		return ret;
	if (!trigger)
		return 0;

	mutex_lock(&mhi_sahara_dump_lock);
	sahara = mhi_sahara_dump_dev;
	if (sahara)
		mhi_sdx55m_dump_sahara_state(sahara->mhi_dev);
	mutex_unlock(&mhi_sahara_dump_lock);

	return sahara ? 0 : -ENODEV;
}

static const struct kernel_param_ops mhi_sahara_dump_ops = {
	.set = mhi_sahara_dump_set,
};
module_param_cb(dump, &mhi_sahara_dump_ops, NULL, 0200);
MODULE_PARM_DESC(dump,
		 "Write 1 to dump SDX55M SBL SAHARA channel/event-ring state to dmesg");

static void mhi_sahara_ul_xfer_cb(struct mhi_device *mhi_dev,
				  struct mhi_result *result)
{
	struct mhi_sahara_dev *sahara = dev_get_drvdata(&mhi_dev->dev);
	struct mhi_sahara_buf *txbuf;
	bool schedule_restart;
	unsigned int restart_delay_ms;
	unsigned int rx_seq;
	u32 cmd;

	txbuf = mhi_sahara_buf_from_data(result->buf_addr);
	schedule_restart = txbuf->restart_after_completion;
	restart_delay_ms = txbuf->restart_delay_ms;
	rx_seq = txbuf->rx_seq;
	cmd = txbuf->cmd;
	kfree(txbuf);
	atomic_inc(&sahara->tx_seq);
	wake_up_interruptible(&sahara->write_wq);

	if (!sahara->is_sahara)
		return;

	if (mhi_sahara_restart_idle_only &&
	    atomic_read(&sahara->rx_seq) != rx_seq)
		schedule_restart = false;

	if (!result->transaction_status && schedule_restart && restart_delay_ms &&
	    test_bit(MHI_SAHARA_OPEN, &sahara->flags) &&
	    test_bit(MHI_SAHARA_RX_REFILL, &sahara->flags)) {
		dev_info(&mhi_dev->dev,
			 "Scheduling SDX55M SAHARA pipe restart after UL cmd=%u delay_ms=%u\n",
			 cmd, restart_delay_ms);
		mod_delayed_work(system_wq, &sahara->sahara_restart,
				 msecs_to_jiffies(restart_delay_ms));
	}
}

static void mhi_sahara_dl_xfer_cb(struct mhi_device *mhi_dev,
				  struct mhi_result *result)
{
	struct mhi_sahara_dev *sahara = dev_get_drvdata(&mhi_dev->dev);
	struct mhi_sahara_buf *rxbuf;
	unsigned long flags;
	bool raw_rx = false;
	u32 cmd;

	rxbuf = mhi_sahara_buf_from_data(result->buf_addr);
	rxbuf->len = result->bytes_xferd;
	rxbuf->pos = 0;

	if (result->transaction_status &&
	    result->transaction_status != -EOVERFLOW)
		goto drop;

	if (!test_bit(MHI_SAHARA_OPEN, &sahara->flags) ||
	    test_bit(MHI_SAHARA_REMOVING, &sahara->flags))
		goto drop;

	if (sahara->raw_rx_remaining) {
		size_t raw_len = min_t(size_t, rxbuf->len,
				       sahara->raw_rx_remaining);

		raw_rx = true;
		cmd = 0;
		sahara->raw_rx_remaining -= raw_len;
		if (raw_len != rxbuf->len) {
			dev_dbg(&mhi_dev->dev,
				"Trimming SAHARA raw command DL len=%zu expected=%zu\n",
				rxbuf->len, raw_len);
			rxbuf->len = raw_len;
		}
		if (!rxbuf->len)
			goto drop_refill;
		goto accept;
	}

	if (!mhi_sahara_rx_valid(rxbuf, rxbuf->len)) {
		dev_dbg(&mhi_dev->dev,
			"Dropping invalid SAHARA pipe DL packet len=%zu status=%d\n",
			rxbuf->len, result->transaction_status);
		goto drop_refill;
	}

	cmd = get_unaligned_le32(rxbuf->data);
	if (cmd == SAHARA_CMD_EXEC_RESP_CMD && rxbuf->len >= 16) {
		sahara->raw_rx_remaining = get_unaligned_le32(rxbuf->data + 12);
		if (sahara->raw_rx_remaining)
			dev_dbg(&mhi_dev->dev,
				"Expecting SAHARA raw command DL bytes=%zu\n",
				sahara->raw_rx_remaining);
	}

accept:
	atomic_inc(&sahara->rx_seq);

	if ((cmd != SAHARA_HELLO_CMD || mhi_sahara_cancel_restart_on_hello) &&
	    (cmd != SAHARA_DONE_RESP_CMD ||
	     mhi_sahara_cancel_restart_on_done_resp) &&
	    cancel_delayed_work(&sahara->sahara_restart))
		dev_info(&mhi_dev->dev,
			 "Canceled pending SDX55M SAHARA pipe restart after DL packet\n");

	if (raw_rx)
		dev_dbg(&mhi_dev->dev,
			"Forwarding SAHARA raw command DL len=%zu remaining=%zu\n",
			rxbuf->len, sahara->raw_rx_remaining);

	spin_lock_irqsave(&sahara->rx_lock, flags);
	list_add_tail(&rxbuf->node, &sahara->rx_ready);
	sahara->rx_ready_count++;
	spin_unlock_irqrestore(&sahara->rx_lock, flags);

	wake_up_interruptible(&sahara->read_wq);
	mhi_sahara_rx_budget_inc(sahara);
	return;

drop_refill:
	kfree(rxbuf);
	mhi_sahara_rx_budget_inc(sahara);
	return;

drop:
	kfree(rxbuf);
}

static int mhi_sahara_probe(struct mhi_device *mhi_dev,
			    const struct mhi_device_id *id)
{
	struct mhi_sahara_dev *sahara;
	const char *pipe_name;
	int ret;

	sahara = kzalloc_obj(*sahara, GFP_KERNEL);
	if (!sahara)
		return -ENOMEM;

	sahara->mhi_dev = mhi_dev;
	mutex_init(&sahara->state_lock);
	mutex_init(&sahara->read_lock);
	mutex_init(&sahara->tx_lock);
	spin_lock_init(&sahara->rx_lock);
	INIT_LIST_HEAD(&sahara->rx_ready);
	init_waitqueue_head(&sahara->read_wq);
	init_waitqueue_head(&sahara->write_wq);
	INIT_WORK(&sahara->rx_refill, mhi_sahara_refill_work);
	INIT_DELAYED_WORK(&sahara->event_poll, mhi_sahara_event_poll_work);
	INIT_DELAYED_WORK(&sahara->sahara_restart, mhi_sahara_restart_work);

	if (!mhi_sahara_is_sdx55m(sahara)) {
		ret = -ENODEV;
		goto err_free;
	}

	pipe_name = mhi_sdx55m_pipe_name(mhi_dev);
	sahara->is_sahara = !strcmp(mhi_dev->name, "SAHARA");
	sahara->miscdev.minor = MISC_DYNAMIC_MINOR;
	sahara->miscdev.name = pipe_name;
	sahara->miscdev.fops = &mhi_sahara_fops;
	sahara->miscdev.parent = &mhi_dev->dev;
	sahara->miscdev.mode = 0600;

	dev_set_drvdata(&mhi_dev->dev, sahara);
	ret = misc_register(&sahara->miscdev);
	if (ret)
		goto err_drvdata;

	if (sahara->is_sahara) {
		mutex_lock(&mhi_sahara_dump_lock);
		mhi_sahara_dump_dev = sahara;
		mutex_unlock(&mhi_sahara_dump_lock);
	}

	dev_info(&mhi_dev->dev, "registered Android-like %s pipe /dev/%s\n",
		 mhi_dev->name, pipe_name);
	return 0;

err_drvdata:
	dev_set_drvdata(&mhi_dev->dev, NULL);
err_free:
	kfree(sahara);
	return ret;
}

static void mhi_sahara_remove(struct mhi_device *mhi_dev)
{
	struct mhi_sahara_dev *sahara = dev_get_drvdata(&mhi_dev->dev);

	mutex_lock(&mhi_sahara_dump_lock);
	if (mhi_sahara_dump_dev == sahara)
		mhi_sahara_dump_dev = NULL;
	mutex_unlock(&mhi_sahara_dump_lock);

	set_bit(MHI_SAHARA_REMOVING, &sahara->flags);
	clear_bit(MHI_SAHARA_OPEN, &sahara->flags);
	clear_bit(MHI_SAHARA_RX_REFILL, &sahara->flags);
	wake_up_interruptible(&sahara->read_wq);
	wake_up_interruptible(&sahara->write_wq);

	misc_deregister(&sahara->miscdev);
	cancel_delayed_work_sync(&sahara->sahara_restart);
	cancel_delayed_work_sync(&sahara->event_poll);
	cancel_work_sync(&sahara->rx_refill);

	mutex_lock(&sahara->state_lock);
	if (test_bit(MHI_SAHARA_PREPARED, &sahara->flags)) {
		mhi_unprepare_from_transfer(mhi_dev);
		clear_bit(MHI_SAHARA_PREPARED, &sahara->flags);
	}
	mutex_unlock(&sahara->state_lock);

	mhi_sahara_purge_ready(sahara);
	dev_set_drvdata(&mhi_dev->dev, NULL);
	kfree(sahara);
}

static const struct mhi_device_id mhi_sahara_match_table[] = {
	{ .chan = "SAHARA" },
	{ .chan = "EFS" },
	{}
};
MODULE_DEVICE_TABLE(mhi, mhi_sahara_match_table);

static struct mhi_driver mhi_sahara_driver = {
	.id_table = mhi_sahara_match_table,
	.probe = mhi_sahara_probe,
	.remove = mhi_sahara_remove,
	.ul_xfer_cb = mhi_sahara_ul_xfer_cb,
	.dl_xfer_cb = mhi_sahara_dl_xfer_cb,
	.driver = {
		.name = "mhi_sahara",
	},
};
module_mhi_driver(mhi_sahara_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MHI SAHARA/EFS pipe driver for SDX55M image loading");
