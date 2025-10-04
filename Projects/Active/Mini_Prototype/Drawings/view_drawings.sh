#!/bin/bash
# Quick viewer for technical drawings

echo "Opening technical drawings..."

# Try different viewers in order of preference
if command -v firefox &> /dev/null; then
    firefox '/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/technical_drawings.html' &
elif command -v chromium-browser &> /dev/null; then
    chromium-browser '/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/technical_drawings.html' &
elif command -v google-chrome &> /dev/null; then
    google-chrome '/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/technical_drawings.html' &
else
    echo "No web browser found. Please open technical_drawings.html manually."
fi

# Also open image viewer for individual images
if command -v eog &> /dev/null; then
    eog '/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/isometric_view.png' &
fi

echo "Technical drawings opened in browser and image viewer."
