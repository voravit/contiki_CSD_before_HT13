CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)

#UIP_CONF_IPV6=1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

#COFFEE = 0
#WITH_SERIAL_LINE_INPUT=1
TARGET=lpc17xx

nuke: clean
	-rm -rf $(CONTIKI_PROJECT).hex
	-rm -rf $(CONTIKI_PROJECT).bin
	-rm -rf $(CONTIKI_PROJECT).$(TARGET)
	-rm -rf obj_$(TARGET)

flash: all
	-arm-none-eabi-objcopy $(CONTIKI_PROJECT).$(TARGET) -O ihex $(CONTIKI_PROJECT).hex
	-lpc21isp -hex $(CONTIKI_PROJECT).hex /dev/ttyUSB0 38400 12000
