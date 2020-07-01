MAKEFILE_DIR := $(shell cd $(dir $(lastword $(MAKEFILE_LIST))); pwd)

help:
	@echo "the Jetson Nano Mouse device driver installer"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

build: ## build the Jetson Nano Mouse kernel module, rtmouse.ko
	cd $(MAKEFILE_DIR)/drivers/rtmouse && make rtmouse.ko

clean: ## clean the object files created while building the kernel module
	cd $(MAKEFILE_DIR)/drivers/rtmouse && make clean

install: build ## install rtmouse.ko and set auto load
	cp 50-rtmouse.rules /etc/udev/rules.d/
	cp $(MAKEFILE_DIR)/drivers/rtmouse/rtmouse.ko /lib/modules/`uname -r`/kernel/drivers/misc/
	depmod -A
	modprobe rtmouse
	echo rtmouse | sudo tee /etc/modules-load.d/rtmouse.conf > /dev/null

uninstall: ## remove rtmouse.ko and un-set auto load
	-modprobe -r rtmouse
	rm /etc/udev/rules.d/50-rtmouse.rules
	rm /etc/modules-load.d/rtmouse.conf
	rm /lib/modules/`uname -r`/kernel/drivers/misc/rtmouse.ko

insmod: build ## insmod rtmouse.ko
	sudo insmod $(MAKEFILE_DIR)/drivers/rtmouse/rtmouse.ko
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
