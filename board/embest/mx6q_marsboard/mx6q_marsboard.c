/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6q_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |        \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |          \
    PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |        \
    PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |          \
    PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#ifdef CONFIG_FEC_MXC
#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |     \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |     \
    PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)
#endif

#ifdef CONFIG_SYS_USE_EIMNOR
#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |      \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |       \
    PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)
#endif

#ifdef CONFIG_MXC_SPI
#define SPI_PAD_CTRL (PAD_CTL_HYS |             \
    PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |     \
    PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)
#endif

#ifdef CONFIG_I2C_MXC
#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |        \
    PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |       \
    PAD_CTL_DSE_40ohm | PAD_CTL_HYS |           \
    PAD_CTL_ODE | PAD_CTL_SRE_FAST)
#endif

int dram_init(void)
{
    gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

    return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
    MX6_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart2_pads[] = {
    MX6_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

//iomux_v3_cfg_t const uart3_pads[] = {
//    MX6_PAD_EIM_D25__UART3_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
//    MX6_PAD_EIM_D24__UART3_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
//};

iomux_v3_cfg_t const uart4_pads[] = {
    MX6_PAD_KEY_COL0__UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW0__UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart5_pads[] = {
    MX6_PAD_KEY_COL1__UART5_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
    MX6_PAD_KEY_ROW1__UART5_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};


#if defined(CONFIG_I2C_MXC)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, SGTL5000 */
struct i2c_pads_info i2c_pad_info0 = {
    .scl = {
        .i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
        .gpio_mode = MX6_PAD_EIM_D21__GPIO_3_21 | PC,
        .gp = IMX_GPIO_NR(3, 21)
    },
    .sda = {
        .i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
        .gpio_mode = MX6_PAD_EIM_D28__GPIO_3_28 | PC,
        .gp = IMX_GPIO_NR(3, 28)
    }
};

/* I2C2 Camera, MIPI */
struct i2c_pads_info i2c_pad_info1 = {
    .scl = {
        .i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
        .gpio_mode = MX6_PAD_KEY_COL3__GPIO_4_12 | PC,
        .gp = IMX_GPIO_NR(4, 12)
    },
    .sda = {
        .i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
        .gpio_mode = MX6_PAD_KEY_ROW3__GPIO_4_13 | PC,
        .gp = IMX_GPIO_NR(4, 13)
    }
};

/* I2C3, J15 - RGB connector */
struct i2c_pads_info i2c_pad_info2 = {
    .scl = {
        .i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
        .gpio_mode = MX6_PAD_GPIO_5__GPIO_1_5 | PC,
        .gp = IMX_GPIO_NR(1, 5)
    },
    .sda = {
        .i2c_mode = MX6_PAD_GPIO_16__I2C3_SDA | PC,
        .gpio_mode = MX6_PAD_GPIO_16__GPIO_7_11 | PC,
        .gp = IMX_GPIO_NR(7, 11)
    }
};
#endif


iomux_v3_cfg_t const usdhc2_pads[] = {
    MX6_PAD_SD2_CLK__USDHC2_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_CMD__USDHC2_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT0__USDHC2_DAT0   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT1__USDHC2_DAT1   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT2__USDHC2_DAT2   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD2_DAT3__USDHC2_DAT3   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_GPIO_4__GPIO_1_4        | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc3_pads[] = {
    MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
    MX6_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
    MX6_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const led_pads[] = {
    MX6_PAD_EIM_D17__GPIO_3_17    | MUX_PAD_CTRL(NO_PAD_CTRL),    /* LED_WiFi */
    MX6_PAD_EIM_D18__GPIO_3_18    | MUX_PAD_CTRL(NO_PAD_CTRL),    /* LED_Power/Work */
    MX6_PAD_EIM_D16__GPIO_3_16    | MUX_PAD_CTRL(NO_PAD_CTRL),    /* LED_CAN2 */
    MX6_PAD_KEY_COL2__GPIO_4_10   | MUX_PAD_CTRL(NO_PAD_CTRL),    /* LED_CAN1 */
    MX6_PAD_SD3_RST__GPIO_7_8     | MUX_PAD_CTRL(NO_PAD_CTRL),    /* LED_GPS/GPRS */
};

#if defined(CONFIG_FEC_MXC)
iomux_v3_cfg_t const enet_pads1[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC      | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__ENET_RGMII_TXC   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__ENET_RGMII_TD0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__ENET_RGMII_TD1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__ENET_RGMII_TD2   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__ENET_RGMII_TD3   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_EIM_D31__GPIO_3_31              | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
    MX6_PAD_RGMII_RXC__ENET_RGMII_RXC   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__ENET_RGMII_RD0   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__ENET_RGMII_RD1   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__ENET_RGMII_RD2   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__ENET_RGMII_RD3   | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL  | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO            | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC              | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TXC__ENET_RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD0__ENET_RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD1__ENET_RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD2__ENET_RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TD3__ENET_RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL      | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_REF_CLK__ENET_TX_CLK       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RXC__ENET_RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD0__ENET_RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD1__ENET_RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD2__ENET_RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RD3__ENET_RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL      | MUX_PAD_CTRL(ENET_PAD_CTRL),
    /* AR8035 PHY Reset */
    MX6_PAD_EIM_D31__GPIO_3_31              | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif

#ifdef CONFIG_SYS_USE_EIMNOR
iomux_v3_cfg_t eimnor_pads[] = {
    MX6_PAD_EIM_DA0__WEIM_WEIM_DA_A_0    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA1__WEIM_WEIM_DA_A_1    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA2__WEIM_WEIM_DA_A_2    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA3__WEIM_WEIM_DA_A_3    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA4__WEIM_WEIM_DA_A_4    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA5__WEIM_WEIM_DA_A_5    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA6__WEIM_WEIM_DA_A_6    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA7__WEIM_WEIM_DA_A_7    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA8__WEIM_WEIM_DA_A_8    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA9__WEIM_WEIM_DA_A_9    | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA10__WEIM_WEIM_DA_A_10  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA11__WEIM_WEIM_DA_A_11  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL) ,
    MX6_PAD_EIM_DA12__WEIM_WEIM_DA_A_12  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA13__WEIM_WEIM_DA_A_13  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA14__WEIM_WEIM_DA_A_14  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_DA15__WEIM_WEIM_DA_A_15  | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A16__WEIM_WEIM_A_16      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A17__WEIM_WEIM_A_17      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A18__WEIM_WEIM_A_18      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A19__WEIM_WEIM_A_19      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A20__WEIM_WEIM_A_20      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A21__WEIM_WEIM_A_21      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A22__WEIM_WEIM_A_22      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A23__WEIM_WEIM_A_23      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A24__WEIM_WEIM_A_24      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_A25__WEIM_WEIM_A_25      | MUX_PAD_CTRL(WEIM_NOR_PAD_CTRL),
    MX6_PAD_EIM_OE__WEIM_WEIM_OE         | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_RW__WEIM_WEIM_RW         | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_CS0__WEIM_WEIM_CS_0      | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_CS1__WEIM_WEIM_CS_1      | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_OE__WEIM_WEIM_OE         | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_RW__WEIM_WEIM_RW         | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_LBA__WEIM_WEIM_LBA       | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_WAIT__WEIM_WEIM_WAIT     | MUX_PAD_CTRL(NO_PAD_CTRL),
//
//    /* Steer logic */
//    MX6_PAD_EIM_A24__GPIO_5_4            | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_EB1__GPIO_2_29           | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_EB2__GPIO_2_30           | MUX_PAD_CTRL(NO_PAD_CTRL),
    MX6_PAD_EIM_EB3__GPIO_2_31           | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void eimnor_cs_setup(void)
{
    writel(0x00000120, WEIM_BASE_ADDR + 0x090);
    writel(0x00020181, WEIM_BASE_ADDR + 0x000);
    writel(0x00000001, WEIM_BASE_ADDR + 0x004);
    writel(0x0a020000, WEIM_BASE_ADDR + 0x008);
    writel(0x0000c000, WEIM_BASE_ADDR + 0x00c);
    writel(0x0804a240, WEIM_BASE_ADDR + 0x010);
}

static void setup_eimnor(void)
{
    imx_iomux_v3_setup_multiple_pads(eimnor_pads,
            ARRAY_SIZE(eimnor_pads));

    gpio_direction_output(IMX_GPIO_NR(2, 29), 0);
    gpio_direction_output(IMX_GPIO_NR(2, 30), 0);
    gpio_direction_output(IMX_GPIO_NR(2, 31), 0);
    mdelay(2);
    gpio_set_value(IMX_GPIO_NR(2, 29), 1);
    gpio_set_value(IMX_GPIO_NR(2, 30), 1);
    gpio_set_value(IMX_GPIO_NR(2, 31), 1);

    eimnor_cs_setup();
}
#endif

#if defined(CONFIG_FEC_MXC)
static void setup_iomux_enet(void)
{
    imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

    /* Reset AR8035 PHY */
    gpio_direction_output(IMX_GPIO_NR(3, 31) , 0);
    udelay(500);
    gpio_set_value(IMX_GPIO_NR(3, 31), 1);
}
#endif

iomux_v3_cfg_t const usb_pads[] = {
    MX6_PAD_GPIO_17__GPIO_7_12 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
    imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
    imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
//    imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
    imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
    imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
    imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

    /* Reset USB hub */
    gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
    mdelay(2);
    gpio_set_value(IMX_GPIO_NR(7, 12), 1);

    return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
    {USDHC4_BASE_ADDR},
    {USDHC2_BASE_ADDR},
};

#define USDHC2_CD_GPIO  IMX_GPIO_NR(1, 4)

int board_mmc_getcd(struct mmc *mmc)
{
    struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
    int ret = 0;

    switch (cfg->esdhc_base) {
        case USDHC2_BASE_ADDR:
            gpio_direction_input(IMX_GPIO_NR(1, 4));
            ret = !gpio_get_value(USDHC2_CD_GPIO);
            break;
        case USDHC4_BASE_ADDR:
            ret = 1; /* eMMC/uSDHC4 is always present */
            break;
    }

    return ret;
}

int board_mmc_init(bd_t *bis)
{
    s32 status = 0;
    u32 index = 0;

    usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
    usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

    usdhc_cfg[0].max_bus_width = 8;
    usdhc_cfg[1].max_bus_width = 4;


    for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
        switch (index) {
        case 0:
            // FIXME было наоборот, сначала 2, потом 4
            imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
            break;
        case 1:
            imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
            break;
       default:
            printf("Warning: you configured more USDHC controllers"
                   "(%d) then supported by the board (%d)\n",
                   index + 1, CONFIG_SYS_FSL_USDHC_NUM);
            return status;
        }

        status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
    }

    return status;
}
#endif

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t const ecspi1_pads[] = {
    /* SS1 */
    MX6_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void setup_spi(void)
{
    gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
    imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
                     ARRAY_SIZE(ecspi1_pads));
}
#endif

#if defined(CONFIG_FEC_MXC)
int mx6_rgmii_rework(struct phy_device *phydev)
{
    unsigned short val, id1, id2;

    id1 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID1);
    id2 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID2);
    printf("----phy_id= 0x%x%x\n", id1, id2);

    /* To enable AR8035 ouput a 125MHz clk from CLK_25M */
    phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
    phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
    phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

    val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
    val &= 0xffe3;
    val |= 0x18;
    phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

    /* introduce tx clock delay */
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
    val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
    val |= 0x0100;
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

    /* rgmii gtx clock delay */
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0xb);
    val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
    val &= ~0x60;
    val |= 0x20;
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

    /*check phy power*/
    val = phy_read(phydev, MDIO_DEVAD_NONE, 0x0);
    if(val & BMCR_PDOWN)
        phy_write(phydev, MDIO_DEVAD_NONE, 0x0, (val & ~BMCR_PDOWN));


    return 0;
}

int board_phy_config(struct phy_device *phydev)
{
    mx6_rgmii_rework(phydev);

    if (phydev->drv->config)
        phydev->drv->config(phydev);

    return 0;
}

int board_eth_init(bd_t *bis)
{
    int ret;

    setup_iomux_enet();

    ret = cpu_eth_init(bis);
    if (ret)
        printf("FEC MXC: %s:failed\n", __func__);

    return 0;
}
#endif

static void setup_leds(void)
{
    imx_iomux_v3_setup_multiple_pads(led_pads,
                     ARRAY_SIZE(led_pads));

    /* LED_WiFi */
    gpio_direction_output(IMX_GPIO_NR(3, 17), 1);
    /* LED_Power/Work */
    gpio_direction_output(IMX_GPIO_NR(3, 18), 1);
    /* LED_CAN2 */
    gpio_direction_output(IMX_GPIO_NR(3, 16), 1);
    /* LED_CAN1 */
    gpio_direction_output(IMX_GPIO_NR(4, 10), 1);
    /* LED_GPS/GPRS# */
    gpio_direction_output(IMX_GPIO_NR(7, 8), 0);
}

static void reduce_leds(void)
{
    /* LED_WiFi */
    gpio_set_value(IMX_GPIO_NR(3, 17), 0);
    /* LED_Power/Work */
    gpio_set_value(IMX_GPIO_NR(3, 18), 1);  // оставить включенным
    /* LED_CAN2 */
    gpio_set_value(IMX_GPIO_NR(3, 16), 0);
    /* LED_CAN1 */
    gpio_set_value(IMX_GPIO_NR(4, 10), 0);
    /* LED_GPS/GPRS# */
    gpio_set_value(IMX_GPIO_NR(7, 8), 1);
}

#ifdef CONFIG_CMD_SATA

int setup_sata(void)
{
    struct iomuxc_base_regs *const iomuxc_regs
        = (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
    int ret = enable_sata_clock();
    if (ret)
        return ret;

    clrsetbits_le32(&iomuxc_regs->gpr[13],
            IOMUXC_GPR13_SATA_MASK,
            IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
            |IOMUXC_GPR13_SATA_PHY_7_SATA2M
            |IOMUXC_GPR13_SATA_SPEED_3G
            |(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
            |IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
            |IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
            |IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
            |IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
            |IOMUXC_GPR13_SATA_PHY_1_SLOW);

    return 0;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
    /* Backlight on RGB connector: J15 */
    MX6_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define RGB_BACKLIGHT_GP IMX_GPIO_NR(1, 21)

    /* Backlight on LVDS connector: J6 */
    MX6_PAD_SD1_CMD__GPIO_1_18 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(1, 18)
};

static iomux_v3_cfg_t const rgb_pads[] = {
    MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
    MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
    MX6_PAD_DI0_PIN2__IPU1_DI0_PIN2,
    MX6_PAD_DI0_PIN3__IPU1_DI0_PIN3,
    MX6_PAD_DI0_PIN4__GPIO_4_20,
    MX6_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
    MX6_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
    MX6_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
    MX6_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
    MX6_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
    MX6_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
    MX6_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
    MX6_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
    MX6_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
    MX6_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
    MX6_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
    MX6_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
    MX6_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
    MX6_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
    MX6_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
    MX6_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
    MX6_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
    MX6_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
    MX6_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
    MX6_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
    MX6_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
    MX6_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
    MX6_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
    MX6_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
};

struct display_info_t {
    int bus;
    int addr;
    int pixfmt;
    int (*detect)(struct display_info_t const *dev);
    void    (*enable)(struct display_info_t const *dev);
    struct  fb_videomode mode;
};


static int detect_hdmi(struct display_info_t const *dev)
{
    struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
    return readb(&hdmi->phy_stat0) & HDMI_PHY_HPD;
}

static void enable_hdmi(struct display_info_t const *dev)
{
    struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
    u8 reg;
    printf("%s: setup HDMI monitor\n", __func__);
    reg = readb(&hdmi->phy_conf0);
    reg |= HDMI_PHY_CONF0_PDZ_MASK;
    writeb(reg, &hdmi->phy_conf0);

    udelay(3000);
    reg |= HDMI_PHY_CONF0_ENTMDS_MASK;
    writeb(reg, &hdmi->phy_conf0);
    udelay(3000);
    reg |= HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
    writeb(reg, &hdmi->phy_conf0);
    writeb(HDMI_MC_PHYRSTZ_ASSERT, &hdmi->mc_phyrstz);
}

static int detect_i2c(struct display_info_t const *dev)
{
    return ((0 == i2c_set_bus_num(dev->bus))
        &&
        (0 == i2c_probe(dev->addr)));
}

static void enable_lvds(struct display_info_t const *dev)
{
    struct iomuxc *iomux = (struct iomuxc *)
                IOMUXC_BASE_ADDR;
    u32 reg = readl(&iomux->gpr[2]);
    reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
    writel(reg, &iomux->gpr[2]);
    gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
}

static void enable_rgb(struct display_info_t const *dev)
{
    imx_iomux_v3_setup_multiple_pads(
        rgb_pads,
         ARRAY_SIZE(rgb_pads));
    gpio_direction_output(RGB_BACKLIGHT_GP, 1);
}

static struct display_info_t const displays[] = {{
    .bus    = -1,
    .addr   = 0,
    .pixfmt = IPU_PIX_FMT_RGB24,
    .detect = detect_hdmi,
    .enable = enable_hdmi,
    .mode   = {
        .name           = "HDMI",
        .refresh        = 60,
        .xres           = 1024,
        .yres           = 768,
        .pixclock       = 15385,
        .left_margin    = 220,
        .right_margin   = 40,
        .upper_margin   = 21,
        .lower_margin   = 7,
        .hsync_len      = 60,
        .vsync_len      = 10,
        .sync           = FB_SYNC_EXT,
        .vmode          = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 2,
    .addr   = 0x4,
    .pixfmt = IPU_PIX_FMT_LVDS666,
    .detect = detect_i2c,
    .enable = enable_lvds,
    .mode   = {
        .name           = "Hannstar-XGA",
        .refresh        = 60,
        .xres           = 1024,
        .yres           = 768,
        .pixclock       = 15385,
        .left_margin    = 220,
        .right_margin   = 40,
        .upper_margin   = 21,
        .lower_margin   = 7,
        .hsync_len      = 60,
        .vsync_len      = 10,
        .sync           = FB_SYNC_EXT,
        .vmode          = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 2,
    .addr   = 0x38,
    .pixfmt = IPU_PIX_FMT_LVDS666,
    .detect = detect_i2c,
    .enable = enable_lvds,
    .mode   = {
        .name           = "wsvga-lvds",
        .refresh        = 60,
        .xres           = 1024,
        .yres           = 600,
        .pixclock       = 15385,
        .left_margin    = 220,
        .right_margin   = 40,
        .upper_margin   = 21,
        .lower_margin   = 7,
        .hsync_len      = 60,
        .vsync_len      = 10,
        .sync           = FB_SYNC_EXT,
        .vmode          = FB_VMODE_NONINTERLACED
} }, {
    .bus    = 2,
    .addr   = 0x48,
    .pixfmt = IPU_PIX_FMT_RGB666,
    .detect = detect_i2c,
    .enable = enable_rgb,
    .mode   = {
        .name           = "wvga-rgb",
        .refresh        = 57,
        .xres           = 800,
        .yres           = 480,
        .pixclock       = 37037,
        .left_margin    = 40,
        .right_margin   = 60,
        .upper_margin   = 10,
        .lower_margin   = 10,
        .hsync_len      = 20,
        .vsync_len      = 10,
        .sync           = 0,
        .vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
    int i;
    int ret;
    char const *panel = getenv("panel");
    if (!panel) {
        for (i = 0; i < ARRAY_SIZE(displays); i++) {
            struct display_info_t const *dev = displays+i;
            if (dev->detect(dev)) {
                panel = dev->mode.name;
                printf("auto-detected panel %s\n", panel);
                break;
            }
        }
        if (!panel) {
            panel = displays[0].mode.name;
            printf("No panel detected: default to %s\n", panel);
        }
    } else {
        for (i = 0; i < ARRAY_SIZE(displays); i++) {
            if (!strcmp(panel, displays[i].mode.name))
                break;
        }
    }
    if (i < ARRAY_SIZE(displays)) {
        ret = ipuv3_fb_init(&displays[i].mode, 0,
                    displays[i].pixfmt);
        if (!ret) {
            displays[i].enable(displays+i);
            printf("Display: %s (%ux%u)\n",
                   displays[i].mode.name,
                   displays[i].mode.xres,
                   displays[i].mode.yres);
        } else
            printf("LCD %s cannot be configured: %d\n",
                   displays[i].mode.name, ret);
    } else {
        printf("unsupported panel %s\n", panel);
        ret = -EINVAL;
    }
    return (0 != ret);
}

static void setup_display(void)
{
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
    struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
    struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
    struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;

    int reg;

    /* Turn on LDB0,IPU,IPU DI0 clocks */
    reg = __raw_readl(&mxc_ccm->CCGR3);
    reg |=   MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET
        |MXC_CCM_CCGR3_LDB_DI0_MASK;
    writel(reg, &mxc_ccm->CCGR3);

    /* Turn on HDMI PHY clock */
    reg = __raw_readl(&mxc_ccm->CCGR2);
    reg |=  MXC_CCM_CCGR2_HDMI_TX_IAHBCLK_MASK
           |MXC_CCM_CCGR2_HDMI_TX_ISFRCLK_MASK;
    writel(reg, &mxc_ccm->CCGR2);

    /* clear HDMI PHY reset */
    writeb(HDMI_MC_PHYRSTZ_DEASSERT, &hdmi->mc_phyrstz);

    /* set PFD1_FRAC to 0x13 == 455 MHz (480*18)/0x13 */
    writel(ANATOP_PFD_480_PFD1_FRAC_MASK, &anatop->pfd_480_clr);
    writel(0x13<<ANATOP_PFD_480_PFD1_FRAC_SHIFT, &anatop->pfd_480_set);

    /* set LDB0, LDB1 clk select to 011/011 */
    reg = readl(&mxc_ccm->cs2cdr);
    reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
         |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
    reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
          |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
    writel(reg, &mxc_ccm->cs2cdr);

    reg = readl(&mxc_ccm->cscmr2);
    reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
    writel(reg, &mxc_ccm->cscmr2);

    reg = readl(&mxc_ccm->chsccdr);
    reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
        |MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
        |MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
    reg |= (CHSCCDR_CLK_SEL_LDB_DI0
        <<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
          |(CHSCCDR_PODF_DIVIDE_BY_3
        <<MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
          |(CHSCCDR_IPU_PRE_CLK_540M_PFD
        <<MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
    writel(reg, &mxc_ccm->chsccdr);

    reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
         |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
         |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
         |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
         |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
         |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
         |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
         |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
         |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
    writel(reg, &iomux->gpr[2]);

    reg = readl(&iomux->gpr[3]);
    reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
        | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
           <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
    writel(reg, &iomux->gpr[3]);

    /* backlights off until needed */
    imx_iomux_v3_setup_multiple_pads(backlight_pads,
                     ARRAY_SIZE(backlight_pads));
    gpio_direction_input(LVDS_BACKLIGHT_GP);
    gpio_direction_input(RGB_BACKLIGHT_GP);
}
#endif

int board_early_init_f(void)
{
    setup_leds();

    setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
    setup_display();
#endif

    return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
    return 1;
}

int board_init(void)
{
    /* address of boot parameters */
    gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
    setup_spi();
#endif

#ifdef CONFIG_SYS_I2C_SPEED
    setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
    setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
    setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif

#ifdef CONFIG_CMD_SATA
    setup_sata();
#endif

    reduce_leds();

    return 0;
}

int checkboard(void)
{
    puts("Board: MX6Q-MARSBOARD\n");

    return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
    /* 4 bit bus width */
    {"mmc0",    MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
    {"mmc1",    MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
    {NULL,      0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_CMD_BMODE
    add_board_boot_modes(board_boot_modes);
#endif
    return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
/* no external pmic, always ldo_enable */
void ldo_mode_set(int ldo_bypass)
{
    return;
}
#endif
