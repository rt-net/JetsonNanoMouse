obj-m:=rtmouse.o

LINUX_SRC_DIR:=/lib/modules/$(shell uname -r)/build
VERBOSE:=0

ccflags-y += -std=gnu99 -Wall -Wno-declaration-after-statement

rtmouse.ko: rtmouse.c
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) modules

clean:
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) clean

install: rtmouse.ko
	sudo insmod rtmouse.ko
	sleep 1
	-sudo chmod 666 /dev/rtbuzzer*
	-sudo chmod 666 /dev/rtcounter*
	-sudo chmod 666 /dev/rtled*
	-sudo chmod 666 /dev/rtlightsensor*
	-sudo chmod 666 /dev/rtmotor*
	-sudo chmod 666 /dev/rtswitch*

uninstall:
	sudo rmmod rtmouse

_dmesg:
	dmesg -x --color -l emerg,alert,crit,err,info,debug
