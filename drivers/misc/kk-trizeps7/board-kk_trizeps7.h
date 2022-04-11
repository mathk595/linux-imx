/*	--*- c -*--
 * Copyright (C) 2014 Volker Keith  <keith@keith-koep.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_LINUX_KERNEL_ARCH_ARM_MACH_IMX6_KK_TRIZEPS7_H
#define H_LINUX_KERNEL_ARCH_ARM_MACH_IMX6_KK_TRIZEPS7_H

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)

int __init trizeps7_pmic_init(void);
extern unsigned long  trizeps_board_version;
extern unsigned char *trizeps_board_version_str;

#define BOARD_TRIZEPS7_V1R0        0x07010000  // unburned
#define BOARD_TRIZEPS7_V1R1        0x07010100
#define BOARD_TRIZEPS7_V1R2        0x07010200
#define BOARD_TRIZEPS7_V1R3        0x07010300
#define BOARD_TRIZEPS7_DDR3L_VALID 0x80000000
#define BOARD_TRIZEPS7_HAS_DDR3L   0x40000000
#define UNDEFINED                  0xffffffff

#endif	/* H_LINUX_KERNEL_ARCH_ARM_MACH_IMX6_KK_TRIZEPS7_H */

#define TR7_WIFI_SINGLEBAND      0x001
#define TR7_WIFI_DUALBAND        0x002
#define TR7_WIFI_DUALANTENNA     0x004
#define TR7_BLUETOOTH            0x100

#define LESSWIRE_WiBear_DF1      0x102         
#define LESSWIRE_WiBear_DF2      0x106         


typedef struct _trizeps_hw_info 
{

  unsigned long  trizeps_module;
  unsigned long  trizeps_sodimm;
  unsigned long  trizeps_btwlan;
  unsigned long  trizeps_extcon;
  unsigned long  trizeps_ddr3l;  
  unsigned long  trizeps_resetout_gpio;
  unsigned long  trizeps_hw_board_version;
  unsigned long  trizeps_sw_board_version;
  unsigned char *trizeps_board_version_str;
  unsigned long  trizeps_unique_id[2];
  unsigned long  trizeps_numcores;
  unsigned long  trizeps_litevers;    
  unsigned long  trizeps_cpumaxfreq;
  unsigned long  trizeps_temprange;
  u64            trizeps_ramsize;  
  unsigned long  trizeps_fpga;
  unsigned long  trizeps_lvds;
  unsigned long  trizeps_audio;
  unsigned long  trizeps_mcu;
  unsigned long  trizeps_eth;
  unsigned long  trizeps_bootstoreemmc;    
  unsigned long  trizeps_bootstore;  
  unsigned long  hs;
  unsigned long  sw;    
} TRIZEPS_INFO, *PTRIZEPS_INFO;

extern TRIZEPS_INFO TrizepsBoardVersion;

#define Trizeps_GetnResetoutGpio()  (TrizepsBoardVersion.trizeps_resetout_gpio)


int TrizepsHasPCIeDisablePin(void);
int TrizepsBootedFromeMMC(void);

enum KUK_BOOTSTORAGE {
    KUK_BOOTSTORAGE_UNKNOWN = 0,    
    KUK_BOOTSTORAGE_SDCARD,
    KUK_BOOTSTORAGE_EMMCxGB,
    KUK_BOOTSTORAGE_EMMC4GB,
    KUK_BOOTSTORAGE_EMMC8GB,
    KUK_BOOTSTORAGE_EMMC16GB,
    KUK_BOOTSTORAGE_EMMC32GB
};

  
