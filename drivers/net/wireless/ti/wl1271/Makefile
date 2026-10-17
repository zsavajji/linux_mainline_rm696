# SPDX-License-Identifier: GPL-2.0
wl12xx-objs		= main.o cmd.o io.o event.o tx.o rx.o ps.o acx.o \
			  boot.o init.o debugfs.o scan.o
wl12xx-$(CONFIG_NL80211_TESTMODE)	+= testmode.o
obj-$(CONFIG_WL1271_NOKIA)		+= wl12xx.o

wl12xx_sdio-objs	= sdio.o
obj-$(CONFIG_WL1271_NOKIA_SDIO)		+= wl12xx_sdio.o
