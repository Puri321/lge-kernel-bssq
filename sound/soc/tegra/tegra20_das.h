/*
 * tegra20_das.h - Definitions for Tegra20 DAS driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010 - NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TEGRA20_DAS_H__
#define __TEGRA20_DAS_H__

/* Register TEGRA20_DAS_DAP_CTRL_SEL */
#define TEGRA20_DAS_DAP_CTRL_SEL			0x00
#define TEGRA20_DAS_DAP_CTRL_SEL_COUNT			5
#define TEGRA20_DAS_DAP_CTRL_SEL_STRIDE			4
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_MS_SEL_P		31
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_MS_SEL_S		1
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA1_TX_RX_P	30
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA1_TX_RX_S	1
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA2_TX_RX_P	29
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_SDATA2_TX_RX_S	1
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_CTRL_SEL_P		0
#define TEGRA20_DAS_DAP_CTRL_SEL_DAP_CTRL_SEL_S		5

/* Values for field TEGRA20_DAS_DAP_CTRL_SEL_DAP_CTRL_SEL */
#define TEGRA20_DAS_DAP_SEL_DAC1	0
#define TEGRA20_DAS_DAP_SEL_DAC2	1
#define TEGRA20_DAS_DAP_SEL_DAC3	2
#define TEGRA20_DAS_DAP_SEL_DAP1	16
#define TEGRA20_DAS_DAP_SEL_DAP2	17
#define TEGRA20_DAS_DAP_SEL_DAP3	18
#define TEGRA20_DAS_DAP_SEL_DAP4	19
#define TEGRA20_DAS_DAP_SEL_DAP5	20

/* Register TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL */
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL			0x40
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_COUNT		3
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE		4
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA2_SEL_P	28
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA2_SEL_S	4
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA1_SEL_P	24
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA1_SEL_S	4
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_CLK_SEL_P	0
#define TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_CLK_SEL_S	4

/*
 * Values for:
 * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA2_SEL
 * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_SDATA1_SEL
 * TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_DAC_CLK_SEL
 */
#define TEGRA20_DAS_DAC_SEL_DAP1	0
#define TEGRA20_DAS_DAC_SEL_DAP2	1
#define TEGRA20_DAS_DAC_SEL_DAP3	2
#define TEGRA20_DAS_DAC_SEL_DAP4	3
#define TEGRA20_DAS_DAC_SEL_DAP5	4

/*
 * Names/IDs of the DACs/DAPs.
 */

#define TEGRA20_DAS_DAP_ID_1 0
#define TEGRA20_DAS_DAP_ID_2 1
#define TEGRA20_DAS_DAP_ID_3 2
#define TEGRA20_DAS_DAP_ID_4 3
#define TEGRA20_DAS_DAP_ID_5 4

#define TEGRA20_DAS_DAC_ID_1 0
#define TEGRA20_DAS_DAC_ID_2 1
#define TEGRA20_DAS_DAC_ID_3 2

#ifdef CONFIG_PM
#define TEGRA20_DAS_CACHE_SIZE	((((TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL) + (TEGRA20_DAS_DAC_INPUT_DATA_CLK_SEL_STRIDE*TEGRA20_DAS_DAC_ID_3))>>2) + 1)
#endif

//                                                                 
#if defined(CONFIG_LU6500)
	#include <linux/mutex.h>
#endif
//                                                                 

struct tegra20_das {
	struct device *dev;
	void __iomem *regs;
	struct dentry *debug;
#ifdef CONFIG_PM
	u32  reg_cache[TEGRA20_DAS_CACHE_SIZE];
#endif

//                                                                 
#if defined(CONFIG_LU6500)
	struct mutex mlock;
	unsigned int tristate_count;
#endif
//                                                                 

};

#ifdef CONFIG_PM
/* Restores the das registers from cache */
extern int tegra20_das_suspend();
extern int tegra20_das_resume();
#endif
/*
 * Terminology:
 * DAS: Digital audio switch (HW module controlled by this driver)
 * DAP: Digital audio port (port/pins on Tegra device)
 * DAC: Digital audio controller (e.g. I2S or AC97 controller elsewhere)
 * 
 * The Tegra DAS is a mux/cross-bar which can connect each DAP to a specific
 * DAC, or another DAP. When DAPs are connected, one must be the master and
 * one the slave. Each DAC allows selection of a specific DAP for input, to
 * cater for the case where N DAPs are connected to 1 DAC for broadcast
 * output.
 *
 * This driver is dumb; no attempt is made to ensure that a valid routing
 * configuration is programmed.
 */

//                                                                 
#if defined(CONFIG_LU6500)
/*
 * Dynamic Power Enable/Disable Sequence.
 * is_normal : True then Power On / False then Power Off
 *
 */
extern int tegra20_das_power_mode(bool is_normal);
#endif
//                                                                 

/*
 * Connect a DAP to to a DAC
 * dap_id: DAP to connect: TEGRA20_DAS_DAP_ID_*
 * dac_sel: DAC to connect to: TEGRA20_DAS_DAP_SEL_DAC*
 */
extern int tegra20_das_connect_dap_to_dac(int dap_id, int dac_sel);

/*
 * Connect a DAP to to another DAP
 * dap_id: DAP to connect: TEGRA20_DAS_DAP_ID_*
 * other_dap_sel: DAP to connect to: TEGRA20_DAS_DAP_SEL_DAP*
 * master: Is this DAP the master (1) or slave (0)
 * sdata1rx: Is this DAP's SDATA1 pin RX (1) or TX (0)
 * sdata2rx: Is this DAP's SDATA2 pin RX (1) or TX (0)
 */
extern int tegra20_das_connect_dap_to_dap(int dap_id, int other_dap_sel,
					int master, int sdata1rx,
					int sdata2rx);

/*
 * Connect a DAC's input to a DAP
 * (DAC outputs are selected by the DAP)
 * dac_id: DAC ID to connect: TEGRA20_DAS_DAC_ID_*
 * dap_sel: DAP to receive input from: TEGRA20_DAS_DAC_SEL_DAP*
 */
extern int tegra20_das_connect_dac_to_dap(int dac_id, int dap_sel);

extern int tegra20_das_set_tristate(int dap_id, int is_tristate);

#endif
