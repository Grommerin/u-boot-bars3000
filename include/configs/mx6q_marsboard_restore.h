/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q Sabre Lite board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.		See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_MX6

#include "mx6_common.h"

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_L2CACHE_OFF

#define CONFIG_MACH_TYPE            3771

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN       (10 * 1024 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE        UART2_BASE

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS       0
#define CONFIG_SF_DEFAULT_CS        (0 | (IMX_GPIO_NR(2, 30) << 8))
#define CONFIG_SF_DEFAULT_SPEED     20000000
#define CONFIG_SF_DEFAULT_MODE      SPI_MODE_0
#endif

/* I2C Configs */
//#define CONFIG_CMD_I2C
#if defined(CONFIG_CMD_I2C)
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED        100000
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR   0
#define CONFIG_SYS_FSL_USDHC_NUM    1

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_SYS_MMC_ENV_PART     0      /* user partition */

//#define CONFIG_CMD_SATA
/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE  1
#define CONFIG_DWC_AHSATA_PORT_ID   0
#define CONFIG_DWC_AHSATA_BASE_ADDR SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif
//
#define CONFIG_FEC_MXC
#if defined(CONFIG_FEC_MXC)
//#define CONFIG_CMD_PING
//#define CONFIG_CMD_DHCP
//#define CONFIG_CMD_MII
//#define CONFIG_CMD_NET
#define CONFIG_MII
#define IMX_FEC_BASE                ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE         RGMII
#define CONFIG_ETHPRIME             "FEC"
#define CONFIG_FEC_MXC_PHYADDR      4
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

//#define CONFIG_NETMASK              255.255.255.0
//#define CONFIG_IPADDR               192.168.1.102
//#define CONFIG_SERVERIP             _SERVER_IP_ADDR_
//#define CONFIG_GATEWAYIP            _GATEWAY_IP_ADDR_
#endif

/* USB Configs */
//#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_MXC_USB_PORT         1
#define CONFIG_MXC_USB_PORTSC       (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS        0
#endif

/* Miscellaneous commands */
//#define CONFIG_CMD_BMODE

/* Framebuffer and LCD */
//#define CONFIG_VIDEO
#if defined(CONFIG_VIDEO)
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK            260000000
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX           1
#define CONFIG_BAUDRATE             115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BOOTD
#undef CONFIG_CMD_IMLS

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_BOOTDELAY            -2

#define CONFIG_PREBOOT              ""

#define CONFIG_LOADADDR             0x12000000
#define CONFIG_SYS_TEXT_BASE        0x17800000

#define CONFIG_EXTRA_ENV_SETTINGS \
    "bootargs=console=ttymxc1,115200 init=/sbin/init rw root=/dev/mmcblk1p2 rootwait rw quiet\0" \
    "mmcdev=0\0" \
    "mmcupd=1\0" \
    "mmcpart=1\0" \
    "loadcmd=fatload\0" \
    "loadk=${loadcmd} mmc ${mmcdev}:${mmcpart} 0x10800000 /uImage\0" \
    "loaddtb=${loadcmd} mmc ${mmcdev}:${mmcpart} 0x12000000 /imx6q-marsboard.dtb\0" \
    "normalboot=mmc rescan; " \
               "run loadk; " \
               "run loaddtb; " \
               "bootm 0x10800000 - 0x12000000\0" \
    "upd_load=${loadcmd} mmc ${mmcupd}:${mmcpart} 0x10000000 u-boot.imx\0" \
    "upd_prepare=sf probe; " \
                "sf read 0x10000000 0x50000 0x4FC00\0" \
    "upd_erase=echo Erase spi flash; " \
              "sf erase 0x0 0x50000\0" \
    "upd_write=echo Write u-boot to spi flash; " \
              "sf write 0x10000000 0x400 0x4FC00\0" \
    "upd_cmd=run upd_prepare; " \
            "if run upd_load; then " \
                "run upd_erase; " \
                "run upd_write; " \
                "echo U-boot update completed!!!; " \
            "else " \
                "echo U-boot update file not found; " \
            "fi\0" \
    "upd_check=if run upd_load; then " \
                  "run upd_cmd; " \
              "else " \
                  "setenv mmcupd 0; " \
                  "if run upd_load; then " \
                      "run upd_cmd; " \
                  "fi; " \
              "fi\0" \
    "bootcmd=run upd_check; " \
            "run normalboot\0" \
    "upd_rest1=echo Restore device is mmc1;" \
              "if run upd_load; then " \
                  "run upd_cmd; " \
              "else " \
                  "echo No u-boot file on restore device mmc1; " \
                  "run upd_rest0;" \
              "fi\0" \
    "upd_rest0=echo Restore device is mmc0;" \
              "setenv mmcupd 0; " \
              "if run upd_load; then " \
                  "run upd_cmd; " \
              "else " \
                  "echo No u-boot file on restore device mmc0; " \
                  "run upd_restm;" \
              "fi\0" \
    "upd_memmod=mw 0x177fb00c 0x177fb02c\0" \
    "upd_writem=echo Copy u-boot to spi flash; " \
               "sf write 0x177fb000 0x400 0x4FC00\0" \
    "upd_restm=echo Restore u-boot from DRAM; " \
              "sf probe; " \
              "run upd_erase; " \
              "run upd_memmod; " \
              "run upd_writem; " \
              "echo U-boot restore completed!!!\0" \
    "env_clear=echo Clear restore environment; " \
              "setenv env_clear; " \
              "setenv bootcmd_2; " \
              "setenv upd_rest1; " \
              "setenv upd_rest0; " \
              "setenv upd_restm; " \
              "setenv upd_writem; " \
              "setenv upd_memmod; " \
              "setenv mmcupd 1; " \
              "saveenv\0" \
    "bootcmd_mfg=echo Run u-boot restore; " \
                "if mmc dev 1; then " \
                    "run upd_rest1; " \
                "else " \
                    "run upd_rest0; " \
                "fi; " \
                "run env_clear; " \
                "run normalboot\0"

/* Miscellaneous configurable options */
//#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT           "MX6Q_MARSBOARD U-Boot> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE           1024

///* OCOTP Config */
//#define CONFIG_CMD_FUSE
//#ifdef CONFIG_CMD_FUSE
//#define CONFIG_MXC_OCOTP
//#endif

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE           (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS          16
#define CONFIG_SYS_BARGSIZE         CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START    0x10000000
#define CONFIG_SYS_MEMTEST_END      0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH  0x10800000

#define CONFIG_SYS_LOAD_ADDR        CONFIG_LOADADDR
#define CONFIG_SYS_HZ               1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS        1
#define PHYS_SDRAM                  MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE             (1u * 1024 * 1024 * 1024)

#define CONFIG_SYS_SDRAM_BASE       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR    IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE    IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET   (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR     (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE             (1024 + 1024)

//#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_IS_IN_SPI_FLASH

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET           (512 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV      0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET           (768 * 1024)
#define CONFIG_ENV_SECT_SIZE        (8 * 1024)
#define CONFIG_ENV_SPI_BUS          CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS           CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE         CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ       CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_OF_LIBFDT
//#define CONFIG_CMD_BOOTZ

//#ifndef CONFIG_SYS_DCACHE_OFF
//#define CONFIG_CMD_CACHE
//#endif

#endif			       /* __CONFIG_H */
