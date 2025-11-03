.PHONY: help build flash monitor clean

PORT ?= /dev/ttyUSB0

help:
	@echo "Meshtastic ESP32 Node - Heltec V3"
	@echo ""
	@echo "Usage: make [target] [PORT=/dev/ttyUSBx]"
	@echo ""
	@echo "Targets:"
	@echo "  build       - Build firmware"
	@echo "  flash       - Flash firmware to device"
	@echo "  monitor     - Monitor serial output (921600 baud)"
	@echo "  flash-monitor - Flash and monitor in one command"
	@echo "  clean       - Clean build artifacts"
	@echo ""

build:
	@idf.py build

flash:
	@idf.py flash -p $(PORT)

monitor:
	@idf.py monitor -p $(PORT) -b 921600

flash-monitor:
	@idf.py flash -p $(PORT)
	@idf.py monitor -p $(PORT) -b 921600

clean:
	@idf.py fullclean
	@rm -f sdkconfig

distclean: clean
	@rm -rf build/ *.log

# Development targets
menuconfig:
	@idf.py menuconfig

size: build
	@idf.py size