PROJECT = pico-thermometer
BUILD_DIR = bin

CFILES = $(wildcard *.c)
LDFLAGS += -Wl,--print-memory-usage
CFLAGS += -flto
LDFLAGS += -flto
OPT = -Os

# TODO - you will need to edit these two lines!
DEVICE=stm32l011f3p6
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
