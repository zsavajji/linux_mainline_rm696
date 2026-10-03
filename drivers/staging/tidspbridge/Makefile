obj-$(CONFIG_TIDSPBRIDGE)	+= bridgedriver.o

libgen = gen/gh.o
libcore = core/chnl_sm.o core/msg_sm.o core/io_sm.o core/tiomap3430.o \
		core/tiomap3430_pwr.o core/tiomap_io.o \
		core/ue_deh.o core/wdt.o core/dsp-clock.o core/sync.o
libpmgr = pmgr/chnl.o pmgr/io.o pmgr/msg.o pmgr/cod.o pmgr/dev.o pmgr/dspapi.o \
		pmgr/dmm.o pmgr/cmm.o pmgr/dbll.o
librmgr = rmgr/dbdcd.o rmgr/disp.o rmgr/drv.o rmgr/mgr.o rmgr/node.o \
		rmgr/proc.o rmgr/pwr.o rmgr/rmm.o rmgr/strm.o rmgr/dspdrv.o \
		rmgr/nldr.o rmgr/drv_interface.o rmgr/policy.o
libdload = dynload/cload.o dynload/getsection.o dynload/reloc.o \
		 dynload/tramp.o
libhw = hw/hw_mmu.o

bridgedriver-y := $(libgen) $(libcore) $(libpmgr) $(librmgr) \
			$(libdload) $(libhw)

#Machine dependent
ccflags-y += -D_TI_ -D_DB_TIOMAP -DTMS32060 \
		-DTICFG_PROC_VER -DTICFG_EVM_TYPE -DCHNL_SMCLASS \
		-DCHNL_MESSAGES -DUSE_LEVEL_1_MACROS

ccflags-y += -I$(src)/include
ccflags-y += -I$(src)/core
ccflags-y += -I$(src)/pmgr
ccflags-y += -I$(src)/rmgr
ccflags-y += -I$(src)/dynload
ccflags-y += -I$(src)/hw

