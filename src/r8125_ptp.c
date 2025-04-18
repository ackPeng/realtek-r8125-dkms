// SPDX-License-Identifier: GPL-2.0-only
/*
################################################################################
#
# r8125 is the Linux device driver released for Realtek 2.5 Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2024 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include <linux/in.h>
#include <linux/ethtool.h>
#include <linux/rtnetlink.h>

#include "r8125.h"
#include "r8125_ptp.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static inline struct timespec timespec64_to_timespec(const struct timespec64 ts64)
{
        return *(const struct timespec *)&ts64;
}

static inline struct timespec64 timespec_to_timespec64(const struct timespec ts)
{
        return *(const struct timespec64 *)&ts;
}
#endif

static void _rtl8125_phc_gettime(struct rtl8125_private *tp, struct timespec64 *ts64)
{
        unsigned long flags;
        spin_lock_irqsave(&tp->phy_lock, flags);
        //get local time
        RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_LATCHED_LOCAL_TIME | PTP_EXEC_CMD));

        // nanoseconds: 0x6808[29:0]
        ts64->tv_nsec = RTL_R32(tp, PTP_SOFT_CONFIG_Time_NS_8125) & 0x3fffffff;

        // seconds: 0x680C[47:0]
        ts64->tv_sec = RTL_R16(tp, PTP_SOFT_CONFIG_Time_S_8125 + 4);
        ts64->tv_sec <<= 32;
        ts64->tv_sec |= RTL_R32(tp, PTP_SOFT_CONFIG_Time_S_8125);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void _rtl8125_phc_settime(struct rtl8125_private *tp, const struct timespec64 *ts64)
{
        s64 sec = ts64->tv_sec & 0x0000FFFFFFFFFFFF; // 48 bits
        u32 nsec = (ts64->tv_nsec >> 2); // & 0x3FFFFFFF; / 30 bits

        netif_info(tp, drv, tp->dev, "PTP: _phc set time start %lld:%ld\n", sec, ts64->tv_nsec);
        //udelay(50);

        // nanoseconds: 0x6808[29:0]
        RTL_W32(tp, PTP_SOFT_CONFIG_Time_NS_8125, (ts64->tv_nsec >> 2) & 0x3FFFFFFF);
        //udelay(50);
        //printk(KERN_INFO "r8125: PTP _set hwtstamp 6\n");

        // seconds: 0x680C[47:0]
        RTL_W32(tp, PTP_SOFT_CONFIG_Time_S_8125, ts64->tv_sec & 0xFFFFFFFF);
        //udelay(50);
        //printk(KERN_INFO "r8125: PTP _set hwtstamp 7\n");

        RTL_W16(tp, PTP_SOFT_CONFIG_Time_S_8125 + 4, (ts64->tv_sec >> 32));
        //udelay(50);

        //set local time
        RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_SET_LOCAL_TIME | PTP_EXEC_CMD));
        //udelay(50);
        netif_info(tp, drv, tp->dev, "PTP: _phc set time end\n");
}

static void _rtl8125_phc_adjtime(struct rtl8125_private *tp, s64 delta)
{
        unsigned long flags;
        struct timespec64 d;
        bool negative = (delta < 0);
        u64 sec;
        u32 nsec;

        d = ns_to_timespec64((negative) ? -delta : delta);
        nsec = d.tv_nsec;
        sec = d.tv_sec;

        if (negative) {
                nsec = -nsec;
                sec = -sec;
        }

        nsec &= 0x3fffffff;
        sec &= 0x0000ffffffffffff;

        if (negative) {
                nsec |= PTP_SOFT_CONFIG_TIME_NS_NEGATIVE;
                sec |= PTP_SOFT_CONFIG_TIME_S_NEGATIVE;
        }

        // nanoseconds: 0x6808[29:0]
        spin_lock_irqsave(&tp->phy_lock, flags);
        RTL_W32(tp, PTP_SOFT_CONFIG_Time_NS_8125, nsec);

        // seconds: 0x680C[47:0]
        RTL_W32(tp, PTP_SOFT_CONFIG_Time_S_8125, sec);
        RTL_W16(tp, PTP_SOFT_CONFIG_Time_S_8125 + 4, (sec >> 32));

        //adjust local time
        //RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_DRIFT_LOCAL_TIME | PTP_EXEC_CMD));
        RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_SET_LOCAL_TIME | PTP_EXEC_CMD));
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static int rtl8125_phc_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);

        netif_info(tp, drv, tp->dev, "PTP: phc adjust time\n");
        _rtl8125_phc_adjtime(tp, delta);
        return 0;
}

/*
1ppm means every 125MHz plus 125Hz. It also means every 8ns minus 8ns*10^(-6)
1ns=2^30 sub_ns
8ns*10^(-6) = 8 * 2^30 sub_ns * 10^(-6) = 2^33 sub_ns * 10^(-6) = 8590 = 0x218E sub_ns
1ppb means every 125MHz plus 0.125Hz. It also means every 8ns minus 8ns*10^(-9)
1ns=2^30 sub_ns
8ns*10^(-9) = 8 * 2^30 sub_ns * 10^(-9) = 2^33 sub_ns * 10^(-9) = 8.59 sub_ns = 9 sub_ns
*/
static void _rtl8125_phc_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);
        unsigned long flags;        
        bool negative = false;
        u32 sub_ns;

        netif_info(tp, drv, tp->dev, "PTP: _phc adjust freq\n");

        if (ppb < 0) {
                negative = true;
                ppb = -ppb;
        }

        sub_ns = ppb * 9;
        if (negative) {
                sub_ns = -sub_ns;
                sub_ns &= 0x3fffffff;
                sub_ns |= PTP_ADJUST_TIME_NS_NEGATIVE;
        } else
                sub_ns &= 0x3fffffff;

        spin_lock_irqsave(&tp->phy_lock, flags);
        // correction in nanoseconds: 0x6808[29:0]
        RTL_W32(tp, PTP_SOFT_CONFIG_Time_NS_8125, sub_ns);

        //adjust local time
        RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_DRIFT_LOCAL_TIME | PTP_EXEC_CMD));
        //RTL_W16(tp, PTP_TIME_CORRECT_CMD_8125, (PTP_CMD_SET_LOCAL_TIME | PTP_EXEC_CMD));
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)
static int rtl8125_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	s32 ppb = scaled_ppm_to_ppb(scaled_ppm);

        if (ppb > ptp->max_adj || ppb < -ptp->max_adj)
                return -EINVAL;

        _rtl8125_phc_adjfreq(ptp, ppb);
        return 0;
}
#else
static int rtl8125_phc_adjfreq(struct ptp_clock_info *ptp, s32 delta)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);

        netif_info(tp, drv, tp->dev, "PTP: phc adjust freq\n");

        if (delta > ptp->max_adj || delta < -ptp->max_adj)
                return -EINVAL;

        _rtl8125_phc_adjfreq(ptp, delta);
        return 0;
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(6,2,0)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
static int rtl8125_phc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts64,
                               struct ptp_system_timestamp *sts)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);

        netif_info(tp, drv, tp->dev, "PTP: phc get time\n");
        ptp_read_system_prets(sts);
        _rtl8125_phc_gettime(tp, ts64);
        ptp_read_system_postts(sts);
        return 0;
}
#else
static int rtl8125_phc_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts64)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);

        netif_info(tp, drv, tp->dev, "PTP: phc get ts\n");
        _rtl8125_phc_gettime(tp, ts64);
        return 0;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0) */

static int rtl8125_phc_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts64)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);
        unsigned long flags;
        netif_info(tp, drv, tp->dev, "PTP: phc set time\n");
        spin_lock_irqsave(&tp->phy_lock, flags);
        _rtl8125_phc_settime(tp, ts64);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        return 0;
}

static int rtl8125_phc_enable(struct ptp_clock_info *ptp,
                              struct ptp_clock_request *rq, int on)
{
        struct rtl8125_private *tp = container_of(ptp, struct rtl8125_private, ptp_clock_info);
        unsigned long flags;
        u16 ptp_ctrl;

        netif_info(tp, drv, tp->dev, "PTP: phc enable type %x on %d\n", rq->type, on);

        if (rq->type == PTP_CLK_REQ_PPS) {
                spin_lock_irqsave(&tp->phy_lock, flags);
                ptp_ctrl = RTL_R16(tp, PTP_CTRL_8125) & ~BIT_15;
                if (on)
                        ptp_ctrl |= BIT_14;
                else
                        ptp_ctrl &= ~BIT_14;
                RTL_W16(tp, PTP_CTRL_8125, ptp_ctrl);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
                return 0;
        }
        return -EOPNOTSUPP;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,11,0)
int rtl8125_get_ts_info(struct net_device *netdev, struct kernel_ethtool_ts_info *info)
#else
int rtl8125_get_ts_info(struct net_device *netdev, struct ethtool_ts_info *info)
#endif
{
        struct rtl8125_private *tp = netdev_priv(netdev);

        /* we always support timestamping disabled */
        info->rx_filters = BIT(HWTSTAMP_FILTER_NONE);

        if (!is_8125B(tp))
                return ethtool_op_get_ts_info(netdev, info);

        info->so_timestamping =  SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE |
                                 SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_TX_HARDWARE |
                                 SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;

        info->phc_index = (tp->ptp_clock) ? ptp_clock_index(tp->ptp_clock) : -1;
        info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);

        info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_PTP_V2_EVENT) |
                           BIT(HWTSTAMP_FILTER_PTP_V2_L4_EVENT) | BIT(HWTSTAMP_FILTER_PTP_V2_SYNC) |
                           BIT(HWTSTAMP_FILTER_PTP_V2_L4_SYNC) | BIT(HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
                           BIT(HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ);
        //info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_PTP_V2_EVENT);
        return 0;
}

static const struct ptp_clock_info rtl_ptp_clock_info = {
        .owner      = THIS_MODULE,
        .n_alarm    = 0,
        .n_ext_ts   = 0,
        .n_per_out  = 0,
        .n_pins     = 0,
        .pps        = 1,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)
        .adjfine   = rtl8125_ptp_adjfine,
#else
        .adjfreq    = rtl8125_phc_adjfreq,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0) */
        .adjtime    = rtl8125_phc_adjtime,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
        .gettimex64  = rtl8125_phc_gettime,
#else
        .gettime64  = rtl8125_phc_gettime,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0) */
        .settime64  = rtl8125_phc_settime,
        .enable     = rtl8125_phc_enable,
};

static __always_inline void 
rtl8125_ptp_egresstime(struct rtl8125_private * const tp, 
        struct timespec64 * const ts64, const u16 reg_offset)
{
        unsigned long flags;

        spin_lock_irqsave(&tp->phy_lock, flags);
        //rtnl_lock();
        // nanoseconds: [29:0]
        ts64->tv_nsec = rtl8125_mac_ocp_read(tp, PTP_EGRESS_TIME_BASE_NS_8125 + reg_offset + 2);
        ts64->tv_nsec <<= 16;
        ts64->tv_nsec |= rtl8125_mac_ocp_read(tp, PTP_EGRESS_TIME_BASE_NS_8125 + reg_offset);
        ts64->tv_nsec &= 0x3FFFFFFF;

        /* seconds */
        //[47:0]
        ts64->tv_sec = rtl8125_mac_ocp_read(tp, PTP_EGRESS_TIME_BASE_S_8125 + reg_offset + 4);
        ts64->tv_sec <<= 16;
        ts64->tv_sec |= rtl8125_mac_ocp_read(tp, PTP_EGRESS_TIME_BASE_S_8125 + reg_offset + 2);
        ts64->tv_sec <<= 16;
        ts64->tv_sec |= rtl8125_mac_ocp_read(tp, PTP_EGRESS_TIME_BASE_S_8125 + reg_offset);
        ts64->tv_sec &= 0x0000FFFFFFFFFFFF;
        //rtnl_unlock();
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

/* Current TX timestamping is serialized, but with 4 registers it could process 4 skb's in parallel */
static void rtl8125_ptp_tx_hwtstamp(struct rtl8125_private *tp, struct sk_buff * const skb)
{
        //struct sk_buff * const skb = tp->ptp_tx_skb;
        struct skb_shared_hwtstamps shhwtstamps = {0};
        struct timespec64 ts64;
        // R8125 has 4 PTP TX timestamp registers which are used in round-robin
        u16 reg;
#ifdef PTP_ROUNDROBIN
        u16 ptp_tx_reg = tp->ptp_tx_reg;
#endif
        //Bits 10~11 are index of TX Timestamp Register [0:3]
        reg = RTL_R16(tp, 0x2032) & 0x0C00;
        reg >>= 10;
        reg = (reg + 3) & 0x0003;

#ifndef PTP_ROUNDROBIN
        udelay(5);
        rtl8125_ptp_egresstime(tp, &ts64, reg * 16);
#else
        if (unlikely(ptp_tx_reg == 0xFFFF))
                ptp_tx_reg = reg;
        if (unlikely(reg != ptp_tx_reg))
                netif_info(tp, drv, tp->dev, "PTP: TX hwtstamp reg mismatch %d:%d\n", reg, ptp_tx_reg);

        rtl8125_ptp_egresstime(tp, &ts64, reg * 16);
        tp->ptp_tx_reg = (ptp_tx_reg + 1) & (u16) 0x0003;
#endif
        /* Upper 32 bits contain s, lower 32 bits contain ns. */
        shhwtstamps.hwtstamp = ktime_set(ts64.tv_sec, ts64.tv_nsec);

        /* The lock can be cleared early since we use a copy of the skb pointer:
         * other threads can't change it while we're notifying the stack. */
        tp->ptp_tx_skb = NULL;
        clear_bit_unlock(__RTL8125_PTP_TX_IN_PROGRESS, &tp->ptp_state);

        /* Notify the stack and free the skb after we've unlocked */
        skb_tstamp_tx(skb, &shhwtstamps);
        dev_kfree_skb_any(skb);
        netif_info(tp, drv, tp->dev, "PTP: TX hwtstamp %lld:%ld, %d\n", ts64.tv_sec, ts64.tv_nsec, reg);
}

#define RTL8125_PTP_TX_TIMEOUT      (HZ * 15)
static void rtl8125_ptp_tx_work(struct work_struct *work)
{
        struct rtl8125_private *tp = container_of(work, struct rtl8125_private, ptp_tx_work);
        struct sk_buff * const skb = tp->ptp_tx_skb;
        int count = 0;
        unsigned long flags;

        //netif_info(tp, drv, tp->dev, "PTP: tx work start\n");
        //spin_lock_irqsave(&tp->ptp_tx_lock, flags);
        if (unlikely(!skb))
                return;

        
        if (unlikely(time_is_before_jiffies(tp->ptp_tx_start + RTL8125_PTP_TX_TIMEOUT))) {
        //if (time_is_after_jiffies(tp->ptp_tx_start + RTL8125_PTP_TX_TIMEOUT)) {
                //unsigned long flags;
        	netif_info(tp, drv, tp->dev, "PTP: TX work hwtstamp timeout\n");
                dev_kfree_skb_any(tp->ptp_tx_skb);
                tp->ptp_tx_skb = NULL;
                clear_bit_unlock(__RTL8125_PTP_TX_IN_PROGRESS, &tp->ptp_state);
                tp->tx_hwtstamp_timeouts++;
                // Clear the tx valid bit in TSYNCTXCTL register to enable interrupt
                //spin_lock_irqsave(&tp->phy_lock, flags);
                RTL_W8(tp, PTP_ISR_8125, PTP_ISR_TOK | PTP_ISR_TER); // PTP_ISR_TOK ?
                //spin_unlock_irqrestore(&tp->phy_lock, flags);
              	return;
        }
        //*/

	// Check if TX HW stamp is ready, if not check again
        do {
                if (RTL_R8(tp, PTP_ISR_8125) & PTP_ISR_TOK) {
                        spin_lock_irqsave(&tp->phy_lock, flags);
                        RTL_W8(tp, PTP_ISR_8125, PTP_ISR_TOK | PTP_ISR_TER);
                        spin_unlock_irqrestore(&tp->phy_lock, flags);
                        //udelay(1);
                        //RTL_R8(tp, PTP_STATUS_8125); // Undocumented
                        return rtl8125_ptp_tx_hwtstamp(tp, skb);
                        //return;
                }
                udelay(5);

        } while (count++ < 2); // Longer waits don't make sense
        /*
        } while (likely(time_is_after_jiffies(tp->ptp_tx_start + RTL8125_PTP_TX_TIMEOUT)));

        // We ran out of time
        netif_err(tp, drv, tp->dev, "PTP: TX work hwtstamp timeout\n");
        tp->tx_hwtstamp_timeouts++;
        dev_kfree_skb_any(tp->ptp_tx_skb);
        tp->ptp_tx_skb = NULL;
        clear_bit_unlock(__RTL8125_PTP_TX_IN_PROGRESS, &tp->ptp_state);
        // Clear the tx valid bit in TSYNCTXCTL register to enable interrupt
        RTL_W8(tp, PTP_ISR_8125, PTP_ISR_TOK | PTP_ISR_TER); // PTP_ISR_TOK ?
        */
        //netif_info(tp, drv, tp->dev, "PTP: TX work reschedule\n");
        schedule_work(&tp->ptp_tx_work); // reschedule to check later
}

static void rtl8125_configure_hwtstamp(struct rtl8125_private *tp, bool enable)
{
        unsigned long flags;
        
        netif_info(tp, drv, tp->dev, "PTP: cfg hwtstamp %sabled\n", enable ? "en" : "dis");
        spin_lock_irqsave(&tp->phy_lock, flags);

        if (enable) {
                u16 ptp_ctrl;
                struct timespec64 ts64;

                RTL_W8(tp, PTP_ISR_8125, 0xff); //clear ptp isr
                udelay(50);

                // Set ptp source 0:gphy 1:mac
                rtl8125_set_mac_ocp_bit(tp, 0xDC00, BIT_6);
                //rtl8125_mac_ocp_write(tp, 0xDC00, rtl8125_mac_ocp_read(tp, 0xDC00) | BIT_6);
                udelay(50);

                // enable ptp
                //printk(KERN_INFO "r8125: PTP set hwtstamp 2\n");
                ptp_ctrl = (BIT_0 | BIT_3 | BIT_4 | BIT_6 | BIT_10 | BIT_12);
                if (rtl8125_flag_is_set(tp, PtpMasterMode))
                        ptp_ctrl |= BIT_1;
                //printk(KERN_INFO "r8125: PTP set hwtstamp 3\n");

                RTL_W16(tp, PTP_CTRL_8125, ptp_ctrl);
                udelay(50);

                // Set system time
                // if (ktime_to_timespec64_cond(ktime_get_real(), &ts64))
                //	_rtl8125_phc_settime(tp, timespec64_to_timespec(ts64));
                ktime_get_real_ts64(&ts64);
                //printk(KERN_INFO "r8125: PTP set hwtstamp 4\n");
                _rtl8125_phc_settime(tp, &ts64);

#ifdef PTP_ROUNDROBIN
                tp->ptp_tx_reg = 0xFFFF;
#endif
        } else {
                RTL_W16(tp, PTP_CTRL_8125, 0);
                udelay(50);
        }
        spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static long rtl8125_ptp_create_clock(struct rtl8125_private *tp, struct net_device *netdev)
{
        long err;

        if (!IS_ERR_OR_NULL(tp->ptp_clock))
                return 0;

        tp->ptp_clock_info = rtl_ptp_clock_info;
        snprintf(tp->ptp_clock_info.name, sizeof(tp->ptp_clock_info.name),
                 "%pm", netdev->dev_addr);
        tp->ptp_clock_info.max_adj = PTP_MAX_ADJ;
        tp->ptp_clock = ptp_clock_register(&tp->ptp_clock_info, tp->device);
        if (IS_ERR(tp->ptp_clock)) {
                err = PTR_ERR(tp->ptp_clock);
                tp->ptp_clock = NULL;
                netif_err(tp, drv, netdev, "ptp_clock_register failed\n");
                return err;
	}       
        netif_info(tp, drv, netdev, "PTP: registered PHC device on %s\n", netdev->name);

        return 0;
}

void rtl8125_ptp_reset(struct rtl8125_private *tp, struct net_device *netdev)
{
        if (unlikely(!tp->ptp_clock))
                return;

        netif_info(tp, drv, netdev, "PTP: reset PHC clock\n");
        rtl8125_configure_hwtstamp(tp, false);
}

void rtl8125_ptp_init(struct rtl8125_private *tp, struct net_device *netdev)
{
        /* obtain a PTP device, or re-use an existing device */
        if (rtl8125_ptp_create_clock(tp, netdev))
                return;

        //spin_lock_init(&tp->ptp_tx_lock);

        /* we have a clock so we can initialize work now */
        INIT_WORK(&tp->ptp_tx_work, rtl8125_ptp_tx_work);

        /* reset the PTP related hardware bits */
        rtl8125_ptp_reset(tp, netdev);
        netif_info(tp, drv, tp->dev, "PTP: initialized (%s)\n", 
               rtl8125_flag_is_set(tp, PtpMasterMode) ? "master": "slave");
        return;
}

void rtl8125_ptp_suspend(struct rtl8125_private *tp)
{
        if (unlikely(!tp->ptp_clock))
                return;

        netif_info(tp, drv, tp->dev, "PTP: suspend PHC clock\n");
        rtl8125_configure_hwtstamp(tp, false);

        /* ensure that we cancel any pending PTP Tx work item in progress */
        cancel_work_sync(&tp->ptp_tx_work);
}

void rtl8125_ptp_stop(struct rtl8125_private *tp)
{
        netif_info(tp, drv, tp->dev, "PTP: stop PHC clock\n");

        /* first, suspend PTP activity */
        rtl8125_ptp_suspend(tp);

        /* disable the PTP clock device */
        if (tp->ptp_clock) {
                ptp_clock_unregister(tp->ptp_clock);
                tp->ptp_clock = NULL;
                netif_info(tp, drv, tp->dev, "PTP: removed PHC on %s\n", tp->dev->name);
        }
}

int rtl8125_set_tstamp(struct rtl8125_private *tp, struct ifreq *ifr)
{
        struct hwtstamp_config config;
        bool hwtstamp = 0;

        netif_info(tp, drv, tp->dev, "PTP: set ts\n");

        if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
                return -EFAULT;

        if (config.flags)
                return -EINVAL;

        //hwtstamp = config.tx_type;
        //if (hwtstamp > HWTSTAMP_TX_ON) return -ERANGE;
        ///*
        switch (config.tx_type) {
        case HWTSTAMP_TX_ON:
                hwtstamp = 1;
        case HWTSTAMP_TX_OFF:
                break;
        case HWTSTAMP_TX_ONESTEP_SYNC:
        default:
        	netif_info(tp, drv, tp->dev, "PTP: set ts onestep\n");
                return -ERANGE;
        }
        //*/
        switch (config.rx_filter) {
        case HWTSTAMP_FILTER_PTP_V2_EVENT:
        case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
        case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
        case HWTSTAMP_FILTER_PTP_V2_SYNC:
        case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
        case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
        case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
        case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
        case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
                hwtstamp = 1;
        case HWTSTAMP_FILTER_NONE:
                break;
        default:
               	netif_err(tp, drv, tp->dev, "PTP: set ts filter fault %x\n", config.rx_filter);
                return -ERANGE;
        }

        if (tp->hwtstamp_config.tx_type != config.tx_type ||
            tp->hwtstamp_config.rx_filter != config.rx_filter) {
                tp->hwtstamp_config = config;
                rtl8125_configure_hwtstamp(tp, hwtstamp);
        }

	if (copy_to_user(ifr->ifr_data, &config, sizeof(config))) {
        	netif_err(tp, drv, tp->dev, "PTP: set ts copy_to_user fault\n");
		return -EFAULT;
	}
        netif_info(tp, drv, tp->dev, "PTP: exit set ts - hwstamp:%d\n", hwtstamp);
	return 0;
        //return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

inline int rtl8125_get_tstamp(struct rtl8125_private *tp, struct ifreq *ifr)
{
        netif_info(tp, drv, tp->dev, "PTP: get ts\n");
        return copy_to_user(ifr->ifr_data, &tp->hwtstamp_config, sizeof(tp->hwtstamp_config)) ? -EFAULT : 0;
}
/*
int rtl8125_ptp_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
        int ret;

        netif_info(tp, drv, tp->dev, "PTP: ioctl\n");

        switch (cmd) {
#ifdef ENABLE_PTP_SUPPORT
        case SIOCSHWTSTAMP:
                ret = rtl8125_set_tstamp(netdev, ifr);
                break;
        case SIOCGHWTSTAMP:
                ret = rtl8125_get_tstamp(netdev, ifr);
                break;
#endif
        default:
                ret = -EOPNOTSUPP;
                break;
        }

        return ret;
}
*/

inline void rtl8125_rx_ptp_pktstamp(struct rtl8125_private *tp, struct sk_buff *skb,
                             struct RxDescV3 *descv3)
{
        time64_t tv_sec = le32_to_cpu(descv3->RxDescTimeStamp.TimeStampHigh) +
                 ((u64)le32_to_cpu(descv3->RxDescPTPDDWord4.TimeStampHHigh) << 32);
        long tv_nsec = le32_to_cpu(descv3->RxDescTimeStamp.TimeStampLow);
        skb_hwtstamps(skb)->hwtstamp = ktime_set(tv_sec, tv_nsec);
}
