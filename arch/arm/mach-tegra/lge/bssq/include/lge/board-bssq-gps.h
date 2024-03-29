/*
 * arch/arm/mach-tegra/board-x3.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <mach-tegra/board.h>
#include <lge/board-bssq.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#define GPS_RESET_N_GPIO			TEGRA_GPIO_PF5	//45
#define GPS_PWR_ON_GPIO				TEGRA_GPIO_PF6	//46
#define GPS_LNA_SD_GPIO				TEGRA_GPIO_PM0	//96

struct gps_gpio_platform_data {
	unsigned pwron;		/* PWR_ON GPIO */
	unsigned reset_n;	/* RESET_N GPIO */
	unsigned lna_sd;	/* LNA_SD GPIO */
};



extern void bssq_gps_init(void);