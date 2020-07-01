MODULE:= rtmouse
obj-m:= $(MODULE).o
clean-files := *.o *.ko *.mod.[co] *~

LINUX_SRC_DIR:=/lib/modules/$(shell uname -r)/build
VERBOSE:=0
ccflags-y += -std=gnu99 -Wall -Wno-declaration-after-statement

rtmouse.ko: rtmouse.c
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) modules

clean:
	make -C $(LINUX_SRC_DIR) M=$(shell pwd) V=$(VERBOSE) clean
