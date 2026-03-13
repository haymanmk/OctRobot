#!/bin/bash
# Setup script for OctroBot firmware development environment

set -e

echo "=========================================="
echo "OctroBot Firmware Setup"
echo "=========================================="

# Check for existing Zephyr virtual environment
ZEPHYR_VENV="${HOME}/zephyrproject/.venv"

if [ -d "$ZEPHYR_VENV" ]; then
    echo "✓ Found existing Zephyr virtual environment at $ZEPHYR_VENV"
    echo "  Activating..."
    source "$ZEPHYR_VENV/bin/activate"
else
    echo "⚠️  Zephyr virtual environment not found at $ZEPHYR_VENV"
    echo "   Expected location: $ZEPHYR_VENV"
    echo ""
    echo "To create it, run:"
    echo "  python3 -m venv $ZEPHYR_VENV"
    echo "  source $ZEPHYR_VENV/bin/activate"
    echo "  pip install west"
    exit 1
fi

# Check Python
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "✓ Python $PYTHON_VERSION found"

# Install West in the virtual environment if not present
if ! command -v west &> /dev/null; then
    echo "Installing West in virtual environment..."
    pip install west
else
    WEST_VERSION=$(west --version)
    echo "✓ West $WEST_VERSION found"
fi

# Check if ZEPHYR_BASE is set (indicates global Zephyr installation)
if [ -n "$ZEPHYR_BASE" ]; then
    echo ""
    echo "✓ ZEPHYR_BASE is set: $ZEPHYR_BASE"
    echo "  Using existing Zephyr installation (skipping west workspace setup)"
    
    # Verify Zephyr installation exists
    if [ ! -d "$ZEPHYR_BASE" ]; then
        echo "⚠️  Warning: ZEPHYR_BASE points to non-existent directory"
        exit 1
    fi
else
    # Initialize West workspace
    echo ""
    echo "Initializing West workspace..."
    if [ ! -d ".west" ]; then
        west init -l .
        echo "✓ West workspace initialized"
    else
        echo "✓ West workspace already initialized"
    fi

    # Update Zephyr and dependencies
    echo ""
    echo "Downloading Zephyr RTOS and dependencies..."
    echo "(This may take several minutes on first run)"
    west update

    echo ""
    echo "Installing Zephyr Python dependencies..."
    pip install -r zephyr/scripts/requirements.txt
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Virtual environment: $ZEPHYR_VENV"
echo ""
echo "To use this environment in your shell:"
echo "  source $ZEPHYR_VENV/bin/activate"
echo ""
echo "Next steps:"
echo "1. Install Zephyr SDK from: https://github.com/zephyrproject-rtos/sdk-ng/releases"
echo "2. Build the project: west build -b m5stack_atom_lite/esp32/procpu app"
echo "3. Flash to device: west flash"
echo ""
