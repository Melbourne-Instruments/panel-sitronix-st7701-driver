obj-m += panel-sitronix-st7701.o
obj-m += panel-sitronix-st7701.o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH)  M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*
