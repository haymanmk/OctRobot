# Makefile wrapper for OctroBot firmware build commands
# Simplifies common West operations

SOC ?= esp32
CORE ?= procpu
BOARD ?= m5stack_atom_lite/$(SOC)/$(CORE)
APP_DIR ?= app
BUILD_DIR ?= build
ZEPHYR_VENV ?= $(HOME)/zephyrproject/.venv

# Activate virtual environment for all west commands
WEST := . $(ZEPHYR_VENV)/bin/activate && west

.PHONY: help build flash clean pristine menuconfig monitor setup

help:
	@echo "OctroBot Firmware Build Targets:"
	@echo "  make setup      - Initialize West workspace and install dependencies"
	@echo "  make build      - Build firmware for $(BOARD)"
	@echo "  make flash      - Flash firmware to device"
	@echo "  make clean      - Clean build directory"
	@echo "  make pristine   - Remove build directory completely"
	@echo "  make menuconfig - Open Kconfig menu"
	@echo "  make monitor    - Open serial monitor"
	@echo "  make all        - Build and flash"
	@echo ""
	@echo "Variables:"
	@echo "  SOC=$(SOC)"
	@echo "  CORE=$(CORE)"
	@echo "  BOARD=$(BOARD)"
	@echo "  APP_DIR=$(APP_DIR)"

setup:
	@echo "Running setup script..."
	@./setup.sh

build:
	$(WEST) build -b $(BOARD) $(APP_DIR)

flash:
	$(WEST) flash

clean:
	$(WEST) build -t clean

pristine:
	rm -rf $(BUILD_DIR)

menuconfig:
	$(WEST) build -t menuconfig

monitor:
	@echo "Opening serial monitor (Ctrl+A, K, Y to exit)..."
	@screen /dev/ttyUSB0 115200

all: build flash

rebuild: pristine build

# Development shortcuts
.PHONY: bf bfm

bf: build flash

bfm: build flash monitor
