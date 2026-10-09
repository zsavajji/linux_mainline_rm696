# SPDX-License-Identifier: GPL-2.0-only

ramzswap-y	:=	zcomp.o zram_drv.o

ramzswap-$(CONFIG_ZRAM_BACKEND_LZO)		+= backend_lzorle.o backend_lzo.o
ramzswap-$(CONFIG_ZRAM_BACKEND_LZ4)		+= backend_lz4.o
ramzswap-$(CONFIG_ZRAM_BACKEND_LZ4HC)	+= backend_lz4hc.o
ramzswap-$(CONFIG_ZRAM_BACKEND_ZSTD)	+= backend_zstd.o
ramzswap-$(CONFIG_ZRAM_BACKEND_DEFLATE)	+= backend_deflate.o
ramzswap-$(CONFIG_ZRAM_BACKEND_842)		+= backend_842.o

# The module is ramzswap, zram's 2.6.32 name: Harmattan's enable-swap job loads and unloads it
obj-$(CONFIG_ZRAM)	+=	ramzswap.o
