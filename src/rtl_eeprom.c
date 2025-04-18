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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/delay.h>

#include <asm/io.h>

#include "r8125.h"
#include "rtl_eeprom.h"

//-------------------------------------------------------------------
//rtl8125_eeprom_type():
//  tell the eeprom type
//return value:
//  0: the eeprom type is 93C46
//  1: the eeprom type is 93C56 or 93C66
//-------------------------------------------------------------------
bool rtl8125_eeprom_type(struct rtl8125_private *tp)
{
        u16 magic = 0;

        if(RTL_R8(tp, 0xD2) & 0x04)
                //not support
                //tp->eeprom_type = EEPROM_TWSI;
                //tp->eeprom_len = 256;
                goto out_no_eeprom;
        else if(RTL_R32(tp, RxConfig) & RxCfg_9356SEL)
                rtl8125_set_flag(tp, EEPROM_TYPE_93C56);
        else 
                rtl8125_set_flag(tp, EEPROM_TYPE_93C46);

        magic = rtl8125_eeprom_read_sc(tp, 0);

out_no_eeprom:
        return ((magic == 0x8129) || (magic == 0x8128));
}

void rtl8125_eeprom_cleanup(struct rtl8125_private *tp)
{
        u8 x = RTL_R8(tp, Cfg9346) & ~(Cfg9346_EEDI | Cfg9346_EECS);
        RTL_W8(tp, Cfg9346, x);
        rtl8125_raise_clock(tp, &x);
        rtl8125_lower_clock(tp, &x);
}

static int rtl8125_eeprom_cmd_done(struct rtl8125_private *tp)
{
        rtl8125_stand_by(tp);

        for (int i = 0; i < 50000; i++) {
                if (RTL_R8(tp, Cfg9346) & Cfg9346_EEDO) {
                        udelay(RTL_CLOCK_RATE * 2 * 3);
                        return 0;
                }
                udelay(1);
        }
        return -1;
}

//-------------------------------------------------------------------
//rtl8125_eeprom_read_sc():
//  read one word from eeprom
//-------------------------------------------------------------------
u16 rtl8125_eeprom_read_sc(struct rtl8125_private *tp, u16 reg)
{
        u8 addr_sz = rtl8125_flag_is_set(tp, EEPROM_TYPE_93C46) ? 6 : 8;
        u16 data;
        //printk(KERN_INFO "r8125: rtl8125_eeprom_write_sc size:%d\n", addr_sz);

        RTL_W8(tp, Cfg9346, Cfg9346_EEM1 | Cfg9346_EECS);
        rtl8125_shift_out_bits(tp, RTL_EEPROM_READ_OPCODE, 3);
        rtl8125_shift_out_bits(tp, reg, addr_sz);

        data = rtl8125_shift_in_bits(tp);
        rtl8125_eeprom_cleanup(tp);
        RTL_W8(tp, Cfg9346, 0);

        return data;
}

//-------------------------------------------------------------------
//rtl8125_eeprom_write_sc():
//  write one word to a specific address in the eeprom
//-------------------------------------------------------------------
void rtl8125_eeprom_write_sc(struct rtl8125_private *tp, u16 reg, u16 data)
{
        u8 addr_sz = rtl8125_flag_is_set(tp, EEPROM_TYPE_93C46) ? 6 : 8;
        //printk(KERN_INFO "r8125: rtl8125_eeprom_write_sc size:%d\n", addr_sz);
/*
        int w_dummy_addr = 4;

        if (tp->eeprom_type==EEPROM_TYPE_93C46) {
                addr_sz = 6;
                w_dummy_addr = 4;
        } else if (tp->eeprom_type==EEPROM_TYPE_93C56) {
                addr_sz = 8;
                w_dummy_addr = 6;
        }
*/
        RTL_W8(tp, Cfg9346, Cfg9346_EEM1 | Cfg9346_EECS);

        rtl8125_shift_out_bits(tp, RTL_EEPROM_EWEN_OPCODE, 5);
        rtl8125_shift_out_bits(tp, reg, addr_sz - 2);
        rtl8125_stand_by(tp);

        rtl8125_shift_out_bits(tp, RTL_EEPROM_ERASE_OPCODE, 3);
        rtl8125_shift_out_bits(tp, reg, addr_sz);
        if (rtl8125_eeprom_cmd_done(tp) < 0)
                return;
        rtl8125_stand_by(tp);

        rtl8125_shift_out_bits(tp, RTL_EEPROM_WRITE_OPCODE, 3);
        rtl8125_shift_out_bits(tp, reg, addr_sz);
        rtl8125_shift_out_bits(tp, data, 16);
        if (rtl8125_eeprom_cmd_done(tp) < 0)
                return;
        rtl8125_stand_by(tp);

        rtl8125_shift_out_bits(tp, RTL_EEPROM_EWDS_OPCODE, 5);
        rtl8125_shift_out_bits(tp, reg, addr_sz - 2);

        rtl8125_eeprom_cleanup(tp);
        RTL_W8(tp, Cfg9346, 0);
}

void rtl8125_raise_clock(struct rtl8125_private *tp, u8 *x)
{
        *x = *x | Cfg9346_EESK;
        RTL_W8(tp, Cfg9346, *x);
        udelay(RTL_CLOCK_RATE);
}

void rtl8125_lower_clock(struct rtl8125_private *tp, u8 *x)
{
        *x = *x & ~Cfg9346_EESK;
        RTL_W8(tp, Cfg9346, *x);
        udelay(RTL_CLOCK_RATE);
}

void rtl8125_shift_out_bits(struct rtl8125_private *tp, int data, int count)
{
        u8 x = RTL_R8(tp, Cfg9346) & ~(Cfg9346_EEDI | Cfg9346_EEDO);
        int  mask = 0x01 << (count - 1);

        do {
                if (data & mask)
                        x |= Cfg9346_EEDI;
                else
                        x &= ~Cfg9346_EEDI;

                RTL_W8(tp, Cfg9346, x);
                udelay(RTL_CLOCK_RATE);
                rtl8125_raise_clock(tp, &x);
                rtl8125_lower_clock(tp, &x);
                mask = mask >> 1;
        } while(mask);

        x &= ~Cfg9346_EEDI;
        RTL_W8(tp, Cfg9346, x);
}

u16 rtl8125_shift_in_bits(struct rtl8125_private *tp)
{
        u8 x = RTL_R8(tp, Cfg9346) & ~(Cfg9346_EEDI | Cfg9346_EEDO);
        u16 d = 0;

        for (u8 i = 0; i < 16; i++) {
                d = d << 1;
                rtl8125_raise_clock(tp, &x);

                x = RTL_R8(tp, Cfg9346);
                x &= ~Cfg9346_EEDI;

                if (x & Cfg9346_EEDO)
                        d |= 1;

                rtl8125_lower_clock(tp, &x);
        }

        return d;
}

void rtl8125_stand_by(struct rtl8125_private *tp)
{
        u8 x = RTL_R8(tp, Cfg9346);
        x &= ~(Cfg9346_EECS | Cfg9346_EESK);
        RTL_W8(tp, Cfg9346, x);
        udelay(RTL_CLOCK_RATE);

        x |= Cfg9346_EECS;
        RTL_W8(tp, Cfg9346, x);
}

void rtl8125_set_eeprom_sel_low(struct rtl8125_private *tp)
{
        RTL_W8(tp, Cfg9346, Cfg9346_EEM1);
        RTL_W8(tp, Cfg9346, Cfg9346_EEM1 | Cfg9346_EESK);
        udelay(20);
        RTL_W8(tp, Cfg9346, Cfg9346_EEM1);
}
