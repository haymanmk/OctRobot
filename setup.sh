#!/bin/bash
# Setup script for OctroBot firmware development environment

set -e

echo "=========================================="
echo "OctroBot Firmware Setup"
echo "=========================================="

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is required but not found"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "✓ Python $PYTHON_VERSION found"

# Install West if not present
if ! command -v west &> /dev/null; then
    echo "Installing West..."
    pip3 install --user west
    
    # Add ~/.local/bin to PATH if not already there
    if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
        echo ""
        echo "⚠️  Please add ~/.local/bin to your PATH:"
        echo "    export PATH=\"\$HOME/.local/bin:\$PATH\""
        echo ""
        echo "Add this to your ~/.zshrc or ~/.bashrc to make it permanent"
        exit 1
    fi
else
    WEST_VERSION=$(west --version)
    echo "✓ West $WEST_VERSION found"
fi

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
pip3 install -r zephyr/scripts/requirements.txt

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Install Zephyr SDK from: https://github.com/zephyrproject-rtos/sdk-ng/releases"
echo "2. Build the project: west build -b m5stack_atom_lite app"
echo "3. Flash to device: west flash"
echo ""
