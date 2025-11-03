.PHONY: help build flash monitor clean

PORT ?= /dev/ttyUSB0
IDF_PATH ?= $(HOME)/.esp/v6.0/esp-idf

# Parallel build jobs (defaults to number of CPU cores)
JOBS ?= $(shell nproc)

# Enable compiler cache (ccache) by default; set CCACHE=0 to disable
CCACHE ?= 1

# Source ESP-IDF environment before running idf.py commands
SHELL := /bin/bash
IDF_ENV = . $(IDF_PATH)/export.sh > /dev/null 2>&1 &&

help:
	@echo "Meshtastic ESP32 Node - Heltec V3"
	@echo ""
	@echo "Usage: make [target] [PORT=/dev/ttyUSBx]"
	@echo "       make build JOBS=<n>            # set parallel build jobs (default: nproc)"
	@echo "       make build CCACHE=0           # disable ccache for this build"
	@echo ""
	@echo "Targets:"
	@echo "  build       - Build firmware"
	@echo "  flash       - Flash firmware to device"
	@echo "  monitor     - Monitor serial output (921600 baud)"
	@echo "  flash-monitor - Flash and monitor in one command"
	@echo "  clean       - Clean build artifacts"
	@echo ""

build:
	@$(IDF_ENV) IDF_CCACHE_ENABLE=$(CCACHE) CMAKE_BUILD_PARALLEL_LEVEL=$(JOBS) idf.py build

flash:
	@$(IDF_ENV) idf.py flash -p $(PORT)

monitor:
	@$(IDF_ENV) idf.py monitor -p $(PORT) -b 921600

flash-monitor:
	@$(IDF_ENV) idf.py flash -p $(PORT)
	@$(IDF_ENV) idf.py monitor -p $(PORT) -b 921600

clean:
	@$(IDF_ENV) idf.py fullclean
	@rm -f sdkconfig

distclean: clean
	@rm -rf build/ *.log

# Development targets
menuconfig:
	@$(IDF_ENV) idf.py menuconfig

size: build
	@$(IDF_ENV) idf.py size