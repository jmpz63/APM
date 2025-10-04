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
