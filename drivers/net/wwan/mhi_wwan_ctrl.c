// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2021, Linaro Ltd <loic.poulain@linaro.org> */
#include <linux/kernel.h>
#include <linux/mhi.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>

static unsigned int mhi_wwan_sahara_restart_after_ul_ms;
module_param_named(sahara_restart_after_ul_ms,
		   mhi_wwan_sahara_restart_after_ul_ms, uint, 0644);
MODULE_PARM_DESC(sahara_restart_after_ul_ms,
		 "Restart SDX55M SBL SAHARA channels after successful UL completion, disabled by default");

static bool mhi_wwan_sahara_restart_periodic;
module_param_named(sahara_restart_periodic,
		   mhi_wwan_sahara_restart_periodic, bool, 0644);
MODULE_PARM_DESC(sahara_restart_periodic,
		 "Keep repeating SDX55M SBL SAHARA full channel restarts while idle, disabled by default");

static bool mhi_wwan_sahara_restart_resync_db;
module_param_named(sahara_restart_resync_db,
		   mhi_wwan_sahara_restart_resync_db, bool, 0644);
MODULE_PARM_DESC(sahara_restart_resync_db,
		 "Resync SDX55M SBL SAHARA cached doorbells after diagnostic restart");

static bool mhi_wwan_sahara_kick_only;
module_param_named(sahara_kick_only,
		   mhi_wwan_sahara_kick_only, bool, 0644);
MODULE_PARM_DESC(sahara_kick_only,
		 "Use SDX55M SAHARA doorbell kick instead of full channel restart (less destructive, modem does not respond)");

static bool mhi_wwan_sahara_reset_only;
module_param_named(sahara_reset_only,
		   mhi_wwan_sahara_reset_only, bool, 0644);
MODULE_PARM_DESC(sahara_reset_only,
		 "Use SDX55M SAHARA RESET+START CMDs instead of full unprepare/prepare (proven to put modem in SYS_ERROR, do not enable)");

static bool mhi_wwan_sahara_preserve;
module_param_named(sahara_preserve,
		   mhi_wwan_sahara_preserve, bool, 0644);
MODULE_PARM_DESC(sahara_preserve,
		 "Use SDX55M SAHARA ring-preserving restart (serialized RESET/START via mhi core, keep buf_ring + tre_ring)");

static struct mhi_wwan_dev *mhi_wwan_sahara_dev;
static DEFINE_MUTEX(mhi_wwan_sahara_dev_lock);

static int mhi_wwan_sahara_dump_set(const char *val,
				    const struct kernel_param *kp);
static const struct kernel_param_ops mhi_wwan_sahara_dump_ops = {
	.set = mhi_wwan_sahara_dump_set,
};
module_param_cb(sahara_dump, &mhi_wwan_sahara_dump_ops, NULL, 0200);
MODULE_PARM_DESC(sahara_dump,
		 "Write 1 to dump SDX55M SBL SAHARA channel/event-ring state to dmesg");

/* MHI wwan flags */
enum mhi_wwan_flags {
	MHI_WWAN_DL_CAP,
	MHI_WWAN_UL_CAP,
	MHI_WWAN_RX_REFILL,
};

#define MHI_WWAN_MAX_MTU	0x8000

struct mhi_wwan_dev {
	/* Lower level is a mhi dev, upper level is a wwan port */
	struct mhi_device *mhi_dev;
	struct wwan_port *wwan_port;

	/* State and capabilities */
	unsigned long flags;
	size_t mtu;

	/* Protect against concurrent TX and TX-completion (bh) */
	spinlock_t tx_lock;

	/* Protect RX budget and rx_refill scheduling */
	spinlock_t rx_lock;
	struct work_struct rx_refill;
	struct delayed_work event_poll;
	struct delayed_work sahara_restart;

	/* RX budget is initially set to the size of the MHI RX queue and is
	 * used to limit the number of allocated and queued packets. It is
	 * decremented on data queueing and incremented on data release.
	 */
	unsigned int rx_budget;
};

static void mhi_wwan_ctrl_refill_work(struct work_struct *work);

static bool mhi_wwan_is_sahara(struct mhi_wwan_dev *mhiwwan)
{
	struct mhi_controller *cntrl = mhiwwan->mhi_dev->mhi_cntrl;

	return cntrl->name && !strcmp(cntrl->name, "qcom-sdx55m") &&
	       mhiwwan->mhi_dev->name &&
	       !strcmp(mhiwwan->mhi_dev->name, "SAHARA");
}

static int mhi_wwan_sahara_dump_set(const char *val,
				    const struct kernel_param *kp)
{
	struct mhi_wwan_dev *mhiwwan;
	bool trigger;
	int ret;

	ret = kstrtobool(val, &trigger);
	if (ret)
		return ret;
	if (!trigger)
		return 0;

	mutex_lock(&mhi_wwan_sahara_dev_lock);
	mhiwwan = mhi_wwan_sahara_dev;
	if (mhiwwan)
		mhi_sdx55m_dump_sahara_state(mhiwwan->mhi_dev);
	mutex_unlock(&mhi_wwan_sahara_dev_lock);

	return mhiwwan ? 0 : -ENODEV;
}

static bool mhi_wwan_sahara_rx_valid(struct sk_buff *skb)
{
	u32 cmd, len;

	if (skb->len < 8)
		return false;

	cmd = get_unaligned_le32(skb->data);
	len = get_unaligned_le32(skb->data + 4);

	return cmd && len >= 8 && len <= skb->len;
}

static void mhi_wwan_ctrl_event_poll_work(struct work_struct *work)
{
	struct mhi_wwan_dev *mhiwwan;

	mhiwwan = container_of(to_delayed_work(work), struct mhi_wwan_dev,
			      event_poll);

	if (!test_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags))
		return;

	mhi_sdx55m_drain_events(mhiwwan->mhi_dev);
	schedule_delayed_work(&mhiwwan->event_poll, msecs_to_jiffies(20));
}

static void mhi_wwan_ctrl_sahara_restart_work(struct work_struct *work)
{
	struct mhi_wwan_dev *mhiwwan;
	struct mhi_device *mhi_dev;
	int ret;

	mhiwwan = container_of(to_delayed_work(work), struct mhi_wwan_dev,
			      sahara_restart);
	mhi_dev = mhiwwan->mhi_dev;

	if (!test_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags))
		return;

	if (mhi_wwan_sahara_kick_only) {
		dev_info(&mhi_dev->dev,
			 "Kicking SDX55M SAHARA doorbells (no channel restart)\n");
		mhi_sdx55m_kick_sahara_doorbells(mhi_dev);
		return;
	}

	if (mhi_wwan_sahara_reset_only) {
		int rc = mhi_sdx55m_reset_sahara(mhi_dev);

		dev_info(&mhi_dev->dev,
			 "SDX55M SAHARA RESET+START CMDs (no ring realloc), rc=%d\n",
			 rc);
		return;
	}

	if (mhi_wwan_sahara_preserve) {
		int rc = mhi_sdx55m_restart_sahara_preserve(mhi_dev);

		dev_info(&mhi_dev->dev,
			 "SDX55M SAHARA ring-preserving restart, rc=%d\n", rc);
		return;
	}

	spin_lock_bh(&mhiwwan->rx_lock);
	clear_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags);
	spin_unlock_bh(&mhiwwan->rx_lock);

	cancel_delayed_work_sync(&mhiwwan->event_poll);
	cancel_work_sync(&mhiwwan->rx_refill);

	dev_info(&mhi_dev->dev, "Restarting SDX55M SAHARA channels after UL completion\n");
	mhi_unprepare_from_transfer(mhi_dev);

	if (!mhi_wwan_sahara_restart_after_ul_ms)
		return;

	ret = mhi_prepare_for_transfer(mhi_dev);
	if (ret) {
		dev_err(&mhi_dev->dev, "Failed to restart SDX55M SAHARA channels: %d\n",
			ret);
		return;
	}

	if (mhi_wwan_sahara_restart_resync_db)
		mhi_sdx55m_resync_sahara_doorbells(mhi_dev);

	mhiwwan->rx_budget = mhi_get_free_desc_count(mhi_dev, DMA_FROM_DEVICE);
	set_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags);
	mhi_wwan_ctrl_refill_work(&mhiwwan->rx_refill);
	schedule_delayed_work(&mhiwwan->event_poll, msecs_to_jiffies(20));

	if (mhi_wwan_sahara_restart_periodic) {
		dev_info(&mhi_dev->dev,
			 "Re-scheduling periodic SDX55M SAHARA restart delay_ms=%u\n",
			 mhi_wwan_sahara_restart_after_ul_ms);
		mod_delayed_work(system_wq, &mhiwwan->sahara_restart,
				 msecs_to_jiffies(mhi_wwan_sahara_restart_after_ul_ms));
	}
}

/* Increment RX budget and schedule RX refill if necessary */
static void mhi_wwan_rx_budget_inc(struct mhi_wwan_dev *mhiwwan)
{
	spin_lock_bh(&mhiwwan->rx_lock);

	mhiwwan->rx_budget++;

	if (test_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags))
		schedule_work(&mhiwwan->rx_refill);

	spin_unlock_bh(&mhiwwan->rx_lock);
}

/* Decrement RX budget if non-zero and return true on success */
static bool mhi_wwan_rx_budget_dec(struct mhi_wwan_dev *mhiwwan)
{
	bool ret = false;

	spin_lock_bh(&mhiwwan->rx_lock);

	if (mhiwwan->rx_budget) {
		mhiwwan->rx_budget--;
		if (test_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags))
			ret = true;
	}

	spin_unlock_bh(&mhiwwan->rx_lock);

	return ret;
}

static void __mhi_skb_destructor(struct sk_buff *skb)
{
	/* RX buffer has been consumed, increase the allowed budget */
	mhi_wwan_rx_budget_inc(skb_shinfo(skb)->destructor_arg);
}

static void mhi_wwan_ctrl_refill_work(struct work_struct *work)
{
	struct mhi_wwan_dev *mhiwwan = container_of(work, struct mhi_wwan_dev, rx_refill);
	struct mhi_device *mhi_dev = mhiwwan->mhi_dev;

	while (mhi_wwan_rx_budget_dec(mhiwwan)) {
		struct sk_buff *skb;

		skb = alloc_skb(mhiwwan->mtu, GFP_KERNEL);
		if (!skb) {
			mhi_wwan_rx_budget_inc(mhiwwan);
			break;
		}

		/* To prevent unlimited buffer allocation if nothing consumes
		 * the RX buffers (passed to WWAN core), track their lifespan
		 * to not allocate more than allowed budget.
		 */
		skb->destructor = __mhi_skb_destructor;
		skb_shinfo(skb)->destructor_arg = mhiwwan;

		if (mhi_queue_skb(mhi_dev, DMA_FROM_DEVICE, skb, mhiwwan->mtu, MHI_EOT)) {
			dev_err(&mhi_dev->dev, "Failed to queue buffer\n");
			kfree_skb(skb);
			break;
		}

		if (mhi_wwan_is_sahara(mhiwwan))
			mhi_sdx55m_drain_events(mhi_dev);
	}
}

static int mhi_wwan_ctrl_start(struct wwan_port *port)
{
	struct mhi_wwan_dev *mhiwwan = wwan_port_get_drvdata(port);
	int ret;

	/* Start mhi device's channel(s) */
	ret = mhi_prepare_for_transfer(mhiwwan->mhi_dev);
	if (ret)
		return ret;

	/* Don't allocate more buffers than MHI channel queue size */
	mhiwwan->rx_budget = mhi_get_free_desc_count(mhiwwan->mhi_dev, DMA_FROM_DEVICE);

	/* Add buffers to the MHI inbound queue */
	if (test_bit(MHI_WWAN_DL_CAP, &mhiwwan->flags)) {
		set_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags);
		mhi_wwan_ctrl_refill_work(&mhiwwan->rx_refill);
		if (mhi_wwan_is_sahara(mhiwwan))
			schedule_delayed_work(&mhiwwan->event_poll,
					      msecs_to_jiffies(20));
	}

	return 0;
}

static void mhi_wwan_ctrl_stop(struct wwan_port *port)
{
	struct mhi_wwan_dev *mhiwwan = wwan_port_get_drvdata(port);

	spin_lock_bh(&mhiwwan->rx_lock);
	clear_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags);
	spin_unlock_bh(&mhiwwan->rx_lock);

	cancel_delayed_work_sync(&mhiwwan->sahara_restart);
	cancel_delayed_work_sync(&mhiwwan->event_poll);
	cancel_work_sync(&mhiwwan->rx_refill);

	mhi_unprepare_from_transfer(mhiwwan->mhi_dev);
}

static int mhi_wwan_ctrl_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct mhi_wwan_dev *mhiwwan = wwan_port_get_drvdata(port);
	int ret;

	if (skb->len > mhiwwan->mtu)
		return -EMSGSIZE;

	if (!test_bit(MHI_WWAN_UL_CAP, &mhiwwan->flags))
		return -EOPNOTSUPP;

	/* Queue the packet for MHI transfer and check fullness of the queue */
	spin_lock_bh(&mhiwwan->tx_lock);
	ret = mhi_queue_skb(mhiwwan->mhi_dev, DMA_TO_DEVICE, skb, skb->len, MHI_EOT);
	if (mhi_queue_is_full(mhiwwan->mhi_dev, DMA_TO_DEVICE))
		wwan_port_txoff(port);
	spin_unlock_bh(&mhiwwan->tx_lock);

	if (mhi_wwan_is_sahara(mhiwwan))
		mhi_sdx55m_drain_events(mhiwwan->mhi_dev);

	return ret;
}

static const struct wwan_port_ops wwan_pops = {
	.start = mhi_wwan_ctrl_start,
	.stop = mhi_wwan_ctrl_stop,
	.tx = mhi_wwan_ctrl_tx,
};

static void mhi_ul_xfer_cb(struct mhi_device *mhi_dev,
			   struct mhi_result *mhi_result)
{
	struct mhi_wwan_dev *mhiwwan = dev_get_drvdata(&mhi_dev->dev);
	struct wwan_port *port = mhiwwan->wwan_port;
	struct sk_buff *skb = mhi_result->buf_addr;

	dev_dbg(&mhi_dev->dev, "%s: status: %d xfer_len: %zu\n", __func__,
		mhi_result->transaction_status, mhi_result->bytes_xferd);

	/* MHI core has done with the buffer, release it */
	consume_skb(skb);

	/* There is likely new slot available in the MHI queue, re-allow TX */
	spin_lock_bh(&mhiwwan->tx_lock);
	if (!mhi_queue_is_full(mhiwwan->mhi_dev, DMA_TO_DEVICE))
		wwan_port_txon(port);
	spin_unlock_bh(&mhiwwan->tx_lock);

	if (mhi_wwan_is_sahara(mhiwwan) && !mhi_result->transaction_status &&
	    mhi_wwan_sahara_restart_after_ul_ms &&
	    test_bit(MHI_WWAN_RX_REFILL, &mhiwwan->flags)) {
		dev_info(&mhi_dev->dev,
			 "Scheduling SDX55M SAHARA restart after UL completion delay_ms=%u\n",
			 mhi_wwan_sahara_restart_after_ul_ms);
		mod_delayed_work(system_wq, &mhiwwan->sahara_restart,
				 msecs_to_jiffies(mhi_wwan_sahara_restart_after_ul_ms));
	}
}

static void mhi_dl_xfer_cb(struct mhi_device *mhi_dev,
			   struct mhi_result *mhi_result)
{
	struct mhi_wwan_dev *mhiwwan = dev_get_drvdata(&mhi_dev->dev);
	struct wwan_port *port = mhiwwan->wwan_port;
	struct sk_buff *skb = mhi_result->buf_addr;

	dev_dbg(&mhi_dev->dev, "%s: status: %d receive_len: %zu\n", __func__,
		mhi_result->transaction_status, mhi_result->bytes_xferd);

	if (mhi_result->transaction_status &&
	    mhi_result->transaction_status != -EOVERFLOW) {
		kfree_skb(skb);
		return;
	}

	/* MHI core does not update skb->len, do it before forward */
	skb_put(skb, mhi_result->bytes_xferd);

	if (mhi_wwan_is_sahara(mhiwwan)) {
		if (!mhi_wwan_sahara_rx_valid(skb)) {
			dev_dbg(&mhi_dev->dev,
				"Dropping invalid SAHARA DL packet len=%u\n",
				skb->len);
			kfree_skb(skb);
			return;
		}
		if (mhi_wwan_sahara_restart_after_ul_ms &&
		    cancel_delayed_work(&mhiwwan->sahara_restart))
			dev_info(&mhi_dev->dev,
				 "Canceled pending SDX55M SAHARA restart after DL packet\n");
	}

	wwan_port_rx(port, skb);

	/* Do not increment rx budget nor refill RX buffers now, wait for the
	 * buffer to be consumed. Done from __mhi_skb_destructor().
	 */
}

static int mhi_wwan_ctrl_probe(struct mhi_device *mhi_dev,
			       const struct mhi_device_id *id)
{
	struct mhi_controller *cntrl = mhi_dev->mhi_cntrl;
	struct mhi_wwan_dev *mhiwwan;
	struct wwan_port *port;

	mhiwwan = kzalloc_obj(*mhiwwan);
	if (!mhiwwan)
		return -ENOMEM;

	mhiwwan->mhi_dev = mhi_dev;
	mhiwwan->mtu = MHI_WWAN_MAX_MTU;
	INIT_WORK(&mhiwwan->rx_refill, mhi_wwan_ctrl_refill_work);
	INIT_DELAYED_WORK(&mhiwwan->event_poll, mhi_wwan_ctrl_event_poll_work);
	INIT_DELAYED_WORK(&mhiwwan->sahara_restart,
			  mhi_wwan_ctrl_sahara_restart_work);
	spin_lock_init(&mhiwwan->tx_lock);
	spin_lock_init(&mhiwwan->rx_lock);

	if (mhi_dev->dl_chan)
		set_bit(MHI_WWAN_DL_CAP, &mhiwwan->flags);
	if (mhi_dev->ul_chan)
		set_bit(MHI_WWAN_UL_CAP, &mhiwwan->flags);

	dev_set_drvdata(&mhi_dev->dev, mhiwwan);

	/* Register as a wwan port, id->driver_data contains wwan port type */
	port = wwan_create_port(&cntrl->mhi_dev->dev, id->driver_data,
				&wwan_pops, NULL, mhiwwan);
	if (IS_ERR(port)) {
		kfree(mhiwwan);
		return PTR_ERR(port);
	}

	mhiwwan->wwan_port = port;

	if (mhi_wwan_is_sahara(mhiwwan)) {
		mutex_lock(&mhi_wwan_sahara_dev_lock);
		mhi_wwan_sahara_dev = mhiwwan;
		mutex_unlock(&mhi_wwan_sahara_dev_lock);
	}

	return 0;
};

static void mhi_wwan_ctrl_remove(struct mhi_device *mhi_dev)
{
	struct mhi_wwan_dev *mhiwwan = dev_get_drvdata(&mhi_dev->dev);

	mutex_lock(&mhi_wwan_sahara_dev_lock);
	if (mhi_wwan_sahara_dev == mhiwwan)
		mhi_wwan_sahara_dev = NULL;
	mutex_unlock(&mhi_wwan_sahara_dev_lock);

	wwan_remove_port(mhiwwan->wwan_port);
	cancel_delayed_work_sync(&mhiwwan->sahara_restart);
	cancel_delayed_work_sync(&mhiwwan->event_poll);
	cancel_work_sync(&mhiwwan->rx_refill);
	kfree(mhiwwan);
}

static const struct mhi_device_id mhi_wwan_ctrl_match_table[] = {
	{ .chan = "DUN", .driver_data = WWAN_PORT_AT },
	{ .chan = "DUN2", .driver_data = WWAN_PORT_AT },
	{ .chan = "MBIM", .driver_data = WWAN_PORT_MBIM },
	{ .chan = "QMI", .driver_data = WWAN_PORT_QMI },
	{ .chan = "DIAG", .driver_data = WWAN_PORT_QCDM },
	{ .chan = "FIREHOSE", .driver_data = WWAN_PORT_FIREHOSE },
	{ .chan = "NMEA", .driver_data = WWAN_PORT_NMEA },
	{},
};
MODULE_DEVICE_TABLE(mhi, mhi_wwan_ctrl_match_table);

static struct mhi_driver mhi_wwan_ctrl_driver = {
	.id_table = mhi_wwan_ctrl_match_table,
	.remove = mhi_wwan_ctrl_remove,
	.probe = mhi_wwan_ctrl_probe,
	.ul_xfer_cb = mhi_ul_xfer_cb,
	.dl_xfer_cb = mhi_dl_xfer_cb,
	.driver = {
		.name = "mhi_wwan_ctrl",
	},
};

module_mhi_driver(mhi_wwan_ctrl_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MHI WWAN CTRL Driver");
MODULE_AUTHOR("Loic Poulain <loic.poulain@linaro.org>");
