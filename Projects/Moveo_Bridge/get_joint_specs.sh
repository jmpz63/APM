#!/bin/bash
# Joint Specifications Extractor
# Reads authoritative joint ranges from printer.cfg

echo "🤖 BCN3D MOVEO JOINT SPECIFICATIONS (from printer.cfg)"
echo "=================================================="
echo ""

if [ ! -f "printer.cfg" ]; then
    echo "❌ ERROR: printer.cfg not found in current directory"
    exit 1
fi

echo "📐 JOINT RANGES (AUTHORITATIVE SOURCE: printer.cfg):"
echo ""

# Extract Joint 1 specs
echo "🔧 JOINT 1 (BASE ROTATION):"
grep -A 15 "\[manual_stepper joint1\]" printer.cfg | grep -E "(position_min|position_max|position_endstop|gear_ratio)" | sed 's/^/   /'
echo ""

# Extract Joint 2 specs  
echo "🔧 JOINT 2/2b (SHOULDER - DUAL MOTOR):"
grep -A 15 "\[manual_stepper joint2\]" printer.cfg | grep -E "(position_min|position_max|position_endstop|gear_ratio)" | sed 's/^/   /'
echo ""

echo "🚨 SAFETY NOTES:"
echo "   • Joint 2/2b motors MUST move synchronously"
echo "   • Software limits prevent mechanical damage" 
echo "   • Always reference printer.cfg for latest specifications"
echo ""
echo "Last updated: $(date)"
