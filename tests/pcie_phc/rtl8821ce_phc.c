// SPDX-License-Identifier: GPL-2.0
/*
 * rtl8821ce_phc — a PTP Hardware Clock backed by the RTL8821CE's 802.11 MAC TSF,
 * read over PCIe BAR2 MMIO.
 *
 * Prototype demonstrating the claim that "a real PHC becomes tractable on PCIe":
 * the 802.11 TSF (REG_TSFTR, a free-running ~1 MHz MAC counter latched inside
 * the hardware, below the CSMA/queueing layer) is the same clock a PTP NIC would
 * expose. Over USB a PHC gettime would be a ~68 us jittery control transfer; over
 * PCIe it is a plain MMIO load (~us, low jitter), so a genuine /dev/ptpN that
 * ptp4l/phc_ctl can open and discipline is practical.
 *
 * Design: this is a *passive* observer. It does NOT bind the PCI device or claim
 * its BAR — the in-tree rtw88 driver keeps the chip powered and the TSF ticking.
 * We only ioremap BAR2 and read a read-only status register, which is safe to do
 * concurrently. It exposes the TSF as a disciplinable clock via the standard
 * timecounter/cyclecounter machinery (software adjfine/adjtime — the silicon has
 * no hardware frequency knob), so phc2sys/ptp4l can steer it like any PHC.
 *
 * Scope / honesty: this proves the clock-source + gettime half. It does not add
 * TX/RX hardware timestamping to the wifi frames (that belongs in rtw88's
 * data path and, for TX egress, is firmware-limited). A production version folds
 * this into rtw88 and points the netdev's ethtool get_ts_info at this phc_index.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/spinlock.h>
#include <linux/timecounter.h>
#include <linux/workqueue.h>

#define RTL_VENDOR	0x10ec
#define RTL_8821CE	0xc821
#define RTL_BAR		2
#define REG_TSFTR	0x560		/* 64-bit MAC TSF: low @0x560, high @0x564 */
#define CC_SHIFT	20		/* fractional-ns headroom for adjfine (~1 ppb) */

struct rtl_phc {
	struct pci_dev *pdev;
	void __iomem *bar;
	struct cyclecounter cc;
	struct timecounter tc;
	u32 nominal_mult;
	spinlock_t lock;		/* guards tc/cc */
	struct ptp_clock *ptp;
	struct ptp_clock_info info;
	struct delayed_work keepalive;	/* periodic read to bound timecounter drift */
};

static u64 rtl_cc_read(const struct cyclecounter *cc)
{
	struct rtl_phc *p = container_of(cc, struct rtl_phc, cc);
	u32 lo = ioread32(p->bar + REG_TSFTR);
	u32 hi = ioread32(p->bar + REG_TSFTR + 4);

	return ((u64)hi << 32) | lo;
}

static int rtl_gettimex(struct ptp_clock_info *info, struct timespec64 *ts,
			struct ptp_system_timestamp *sts)
{
	struct rtl_phc *p = container_of(info, struct rtl_phc, info);
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&p->lock, flags);
	ptp_read_system_prets(sts);
	ns = timecounter_read(&p->tc);		/* MMIO read happens inside */
	ptp_read_system_postts(sts);
	spin_unlock_irqrestore(&p->lock, flags);

	*ts = ns_to_timespec64(ns);
	return 0;
}

static int rtl_settime(struct ptp_clock_info *info, const struct timespec64 *ts)
{
	struct rtl_phc *p = container_of(info, struct rtl_phc, info);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	timecounter_init(&p->tc, &p->cc, timespec64_to_ns(ts));
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static int rtl_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct rtl_phc *p = container_of(info, struct rtl_phc, info);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	timecounter_adjtime(&p->tc, delta);
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static int rtl_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	struct rtl_phc *p = container_of(info, struct rtl_phc, info);
	bool neg = scaled_ppm < 0;
	unsigned long flags;
	u64 diff;

	if (neg)
		scaled_ppm = -scaled_ppm;
	/* scaled_ppm is ppm * 2^16; diff = nominal_mult * scaled_ppm / (1e6 * 2^16) */
	diff = mul_u64_u64_div_u64(p->nominal_mult, (u64)scaled_ppm,
				   1000000ULL << 16);

	spin_lock_irqsave(&p->lock, flags);
	timecounter_read(&p->tc);		/* accumulate at the old rate first */
	p->cc.mult = neg ? p->nominal_mult - diff : p->nominal_mult + diff;
	spin_unlock_irqrestore(&p->lock, flags);
	return 0;
}

static int rtl_enable(struct ptp_clock_info *info,
		      struct ptp_clock_request *req, int on)
{
	return -EOPNOTSUPP;			/* no PPS / pins on this part */
}

static void rtl_keepalive(struct work_struct *w)
{
	struct rtl_phc *p = container_of(w, struct rtl_phc, keepalive.work);
	unsigned long flags;

	spin_lock_irqsave(&p->lock, flags);
	timecounter_read(&p->tc);		/* bound cyc-delta so mult math can't overflow */
	spin_unlock_irqrestore(&p->lock, flags);
	schedule_delayed_work(&p->keepalive, 30 * HZ);
}

static struct rtl_phc *g_phc;

static int __init rtl_phc_init(void)
{
	struct pci_dev *pdev;
	struct rtl_phc *p;
	resource_size_t phys, len;
	u64 t0, t1;
	int ret;

	pdev = pci_get_device(RTL_VENDOR, RTL_8821CE, NULL);
	if (!pdev) {
		pr_err("rtl8821ce_phc: no %04x:%04x found\n", RTL_VENDOR, RTL_8821CE);
		return -ENODEV;
	}

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		goto err_put;
	}
	p->pdev = pdev;
	spin_lock_init(&p->lock);

	phys = pci_resource_start(pdev, RTL_BAR);
	len = pci_resource_len(pdev, RTL_BAR);
	if (!phys || len < REG_TSFTR + 8) {
		pr_err("rtl8821ce_phc: BAR%d unexpected (start=%pa len=%pa)\n",
		       RTL_BAR, &phys, &len);
		ret = -ENODEV;
		goto err_free;
	}
	/* Deliberately not request_mem_region(): rtw88 owns the BAR; we only read. */
	p->bar = ioremap(phys, len);
	if (!p->bar) {
		ret = -ENOMEM;
		goto err_free;
	}
	p->cc.read = rtl_cc_read;

	/* DEBUG: dump a few registers to confirm the BAR mapping and find a live
	 * counter. 0x00/0xF0 are low MAC regs; 0x560/0x564 the port0 TSF. */
	pr_info("rtl8821ce_phc: regdump 0x00=%08x 0x04=%08x 0xF0=%08x 0x1C0=%08x 0x554=%08x 0x560=%08x 0x564=%08x\n",
		ioread32(p->bar + 0x00), ioread32(p->bar + 0x04),
		ioread32(p->bar + 0xF0), ioread32(p->bar + 0x1C0),
		ioread32(p->bar + 0x554), ioread32(p->bar + 0x560),
		ioread32(p->bar + 0x564));

	/* Liveness: confirm the TSF is actually ticking before we register. */
	t0 = rtl_cc_read(&p->cc);
	mdelay(50);
	t1 = rtl_cc_read(&p->cc);
	pr_info("rtl8821ce_phc: TSF %llu -> %llu us over ~50 ms (advanced %lld)\n",
		t0, t1, (long long)(t1 - t0));
	if (t1 <= t0)
		pr_warn("rtl8821ce_phc: TSF not advancing yet (idle/unassociated?) — registering anyway for debug\n");

	/* TSF ticks at 1 MHz => 1 cycle = 1000 ns. Encode with CC_SHIFT of
	 * fractional headroom so adjfine has ~ppb resolution. */
	p->cc.mask = CYCLECOUNTER_MASK(64);
	p->cc.shift = CC_SHIFT;
	p->cc.mult = 1000U << CC_SHIFT;
	p->nominal_mult = p->cc.mult;
	timecounter_init(&p->tc, &p->cc, ktime_get_real_ns());

	p->info = (struct ptp_clock_info){
		.owner		= THIS_MODULE,
		.name		= "rtl8821ce_tsf",
		.max_adj	= 100000000,	/* software mult; generous */
		.gettimex64	= rtl_gettimex,
		.settime64	= rtl_settime,
		.adjtime	= rtl_adjtime,
		.adjfine	= rtl_adjfine,
		.enable		= rtl_enable,
	};

	p->ptp = ptp_clock_register(&p->info, &pdev->dev);
	if (IS_ERR(p->ptp)) {
		ret = PTR_ERR(p->ptp);
		pr_err("rtl8821ce_phc: ptp_clock_register failed: %d\n", ret);
		goto err_unmap;
	}
	pr_info("rtl8821ce_phc: registered /dev/ptp%d on %s\n",
		ptp_clock_index(p->ptp), pci_name(pdev));

	INIT_DELAYED_WORK(&p->keepalive, rtl_keepalive);
	schedule_delayed_work(&p->keepalive, 30 * HZ);
	g_phc = p;
	return 0;

err_unmap:
	iounmap(p->bar);
err_free:
	kfree(p);
err_put:
	pci_dev_put(pdev);
	return ret;
}

static void __exit rtl_phc_exit(void)
{
	struct rtl_phc *p = g_phc;

	if (!p)
		return;
	cancel_delayed_work_sync(&p->keepalive);
	ptp_clock_unregister(p->ptp);
	iounmap(p->bar);
	pci_dev_put(p->pdev);
	kfree(p);
	g_phc = NULL;
}

module_init(rtl_phc_init);
module_exit(rtl_phc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PTP hardware clock backed by the RTL8821CE 802.11 MAC TSF over PCIe");
MODULE_AUTHOR("devourer");
