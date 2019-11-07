.DEFAULT_GOAL := all

MODULE:= rtmouse
obj-m:= $(MODULE).o
clean-files := *.o *.ko *.mod.[co] *~

LINUX_SRC_DIR:=/lib/modules/$(shell uname -r)/build
MAKEFILE_DIR := $(shell cd $(dir $(lastword $(MAKEFILE_LIST))); pwd)

VERBOSE:=0
ccflags-y += -std=gnu99 -Wall -Wno-declaration-after-statement

help:
	@echo "the Raspberry Pi Mouse device driver installer"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

all: ## build the Raspberry Pi Mouse kernel module, rtmouse.ko
	make rtmouse.ko

rtmouse.ko: rtmouse.c
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) modules

clean: ## remove rtmouse.ko and other object files
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) clean

install: rtmouse.ko ## install rtmouse.ko and set auto load
	cp 50-rtmouse.rules /etc/udev/rules.d/
	cp rtmouse.ko /lib/modules/`uname -r`/
	depmod -A
	modprobe rtmouse
	rtmouse | sudo tee /etc/modules-load.d/rtmouse.conf > /dev/null

uninstall: ## remove rtmouse.ko and un-set auto load
	-modprobe -r rtmouse
	rm /etc/udev/rules.d/50-rtmouse.rules
	rm /etc/modules-load.d/rtmouse.conf
	rm /lib/modules/`uname -r`/rtmouse.ko

insmod: rtmouse.ko ## insmod rtmouse.ko
	sudo insmod rtmouse.ko
	sleep 1
	-sudo chmod 666 /dev/rtbuzzer*
	-sudo chmod 666 /dev/rtcounter*
	-sudo chmod 666 /dev/rtled*
	-sudo chmod 666 /dev/rtlightsensor*
	-sudo chmod 666 /dev/rtmotor*
	-sudo chmod 666 /dev/rtswitch*

rmmod: ## rmmod rtmouse.ko
	sudo rmmod rtmouse

_dmesg:
	dmesg -x --color -l emerg,alert,crit,err,info,debug
