TARGET = my_first_hps
 
ALT_DEVICE_FAMILY ?= soc_cv_av
 
SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
 
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
 
 
CROSS_COMPILE = arm-linux-gnueabihf-
 
CFLAGS=-g -Wall -D$(ALT_DEVICE_FAMILY) -I$(HWLIBS_ROOT)/include/$(ALT_DEVIVE_FAMILY) -I$(HWLIBS_ROOT)/include/ -I $(HWLIBS_ROOT)/include/soc_a10/ 
 
 
 
 
LDFLAGS= -g -Wall
CC = $(CROSS_COMPILE)gcc
ARCH= arm
 
build: $(TARGET)
 
$(TARGET): main.o ADXL345.o
	$(CC) $(LDFLAGS) $^ -o $@
 
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@
 
.PHONY: clean
 
clean:
	rm -f $(TARGET) *.a *.o *~