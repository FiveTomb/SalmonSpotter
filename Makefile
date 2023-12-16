CONTIKI_PROJECT = salmon-spotter
all: $(CONTIKI_PROJECT)

DRIVER_DIR = drivers/

CONTIKI = ../..

# Add the path to the CFLAGS for the headers.
CFLAGS += -I$(DRIVER_DIR)

include $(CONTIKI)/Makefile.include
