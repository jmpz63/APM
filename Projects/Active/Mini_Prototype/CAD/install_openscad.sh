#!/bin/bash
# OpenSCAD Installation and Setup Script
# Complete automation for viewing 3D models

echo "==================================="
echo "OpenSCAD Installation & Setup"
echo "==================================="

# Check if OpenSCAD is already installed
if command -v openscad &> /dev/null; then
    echo "✓ OpenSCAD is already installed"
    openscad --version
else
    echo "Installing OpenSCAD..."
    sudo apt update
    sudo apt install -y openscad
    
    if command -v openscad &> /dev/null; then
        echo "✓ OpenSCAD installed successfully"
        openscad --version
    else
        echo "❌ OpenSCAD installation failed"
        exit 1
    fi
fi

# Create desktop shortcut
DESKTOP_FILE="$HOME/Desktop/Mini_Prototype_3D.desktop"
cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Name=Mini Prototype 3D Model
Comment=Open manufacturing system 3D model
Exec=openscad /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad
Icon=openscad
Terminal=false
Type=Application
Categories=Graphics;3DGraphics;Engineering;
EOF

chmod +x "$DESKTOP_FILE"
echo "✓ Desktop shortcut created: Mini_Prototype_3D.desktop"

# Verify model file exists
MODEL_FILE="/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad"
if [ -f "$MODEL_FILE" ]; then
    echo "✓ 3D model file found: $MODEL_FILE"
else
    echo "❌ 3D model file not found: $MODEL_FILE"
fi

# Create quick access script
SCRIPT_FILE="/home/arm1/APM/open_3d_model.sh"
cat > "$SCRIPT_FILE" << 'EOF'
#!/bin/bash
# Quick access script for 3D model

echo "Opening Mini Prototype 3D Model..."
cd /home/arm1/APM/Projects/Active/Mini_Prototype/CAD
openscad mini_prototype_model.scad &

echo ""
echo "OpenSCAD Controls:"
echo "F5 - Quick Preview"
echo "F6 - Full Render"  
echo "F4 - Reload Model"
echo ""
echo "To export STL:"
echo "File → Export → Export as STL..."
EOF

chmod +x "$SCRIPT_FILE"
echo "✓ Quick access script created: /home/arm1/APM/open_3d_model.sh"

# Test opening the model
echo ""
echo "Testing model opening..."
if openscad --version &> /dev/null; then
    echo "✓ OpenSCAD is ready"
    echo ""
    echo "==================================="
    echo "Setup Complete!"
    echo "==================================="
    echo ""
    echo "To view your 3D model:"
    echo "1. Double-click 'Mini_Prototype_3D' on desktop"
    echo "2. Or run: /home/arm1/APM/open_3d_model.sh"
    echo "3. Or manually: openscad $MODEL_FILE"
    echo ""
    echo "Quick Reference:"
    echo "• F5 = Fast preview"
    echo "• F6 = Full render (required before export)"
    echo "• Edit parameters at top of .scad file"
    echo "• Export → STL for 3D printing"
    echo ""
else
    echo "❌ OpenSCAD test failed"
fi