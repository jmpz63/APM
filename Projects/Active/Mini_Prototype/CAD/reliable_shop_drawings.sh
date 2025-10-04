#!/bin/bash
# Simple and Reliable Shop Drawing Generator
# Focus on what actually works

set -e

echo "========================================="
echo "Simple Shop Drawing Generator"
echo "========================================="

MODEL_FILE="/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad"
OUTPUT_DIR="/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings"

# Create output directory
mkdir -p "$OUTPUT_DIR"
echo "✓ Output directory ready: $OUTPUT_DIR"

# Method 1: Generate high-quality PNG images from different angles
echo "Generating 3D view images for technical reference..."

# Front view (camera looking at -Y direction)
echo "  Creating front view image..."
openscad --camera=0,2000,500,60,0,0,0 --imgsize=1200,800 \
         --projection=orthogonal --render \
         -o "$OUTPUT_DIR/front_view.png" "$MODEL_FILE"

# Top view (camera looking down -Z direction)  
echo "  Creating top view image..."
openscad --camera=0,0,2000,0,0,0,0 --imgsize=1200,800 \
         --projection=orthogonal --render \
         -o "$OUTPUT_DIR/top_view.png" "$MODEL_FILE"

# Side view (camera looking at +X direction)
echo "  Creating side view image..."
openscad --camera=2000,0,500,60,0,90,0 --imgsize=1200,800 \
         --projection=orthogonal --render \
         -o "$OUTPUT_DIR/side_view.png" "$MODEL_FILE"

# Isometric view
echo "  Creating isometric view..."
openscad --camera=1500,1500,1000,60,0,45,0 --imgsize=1200,800 \
         --projection=perspective --render \
         -o "$OUTPUT_DIR/isometric_view.png" "$MODEL_FILE"

echo "✓ All view images generated"

# Method 2: Create a simple HTML technical drawing viewer
echo "Creating HTML technical drawing viewer..."

cat > "$OUTPUT_DIR/technical_drawings.html" << 'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mini Prototype Technical Drawings</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: #f5f5f5; 
        }
        .header { 
            text-align: center; 
            background: white; 
            padding: 20px; 
            border: 2px solid #333; 
            margin-bottom: 20px;
        }
        .drawing-sheet { 
            background: white; 
            padding: 20px; 
            border: 1px solid #333; 
            margin-bottom: 20px;
        }
        .view-grid { 
            display: grid; 
            grid-template-columns: 1fr 1fr; 
            gap: 20px; 
            margin-bottom: 20px;
        }
        .view { 
            border: 1px solid #666; 
            padding: 10px; 
            text-align: center;
        }
        .view img { 
            max-width: 100%; 
            height: auto; 
            border: 1px solid #ccc;
        }
        .view-title { 
            font-weight: bold; 
            margin-bottom: 10px; 
            background: #e0e0e0; 
            padding: 5px;
        }
        .title-block { 
            background: #f0f0f0; 
            border: 2px solid #333; 
            padding: 15px; 
            margin-top: 20px;
        }
        .notes { 
            background: #fff9c4; 
            border: 1px solid #ccc; 
            padding: 15px; 
            margin-top: 20px;
        }
        .specs-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .specs-table th, .specs-table td {
            border: 1px solid #333;
            padding: 8px;
            text-align: left;
        }
        .specs-table th {
            background-color: #e0e0e0;
        }
        @media print {
            body { margin: 0; background: white; }
            .drawing-sheet { page-break-after: always; }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>MINI WALL PANEL MANUFACTURING SYSTEM</h1>
        <h2>Technical Drawings and Specifications</h2>
        <p><strong>Drawing Number:</strong> MP-001 | <strong>Revision:</strong> A | <strong>Date:</strong> 2025-10-04</p>
    </div>

    <div class="drawing-sheet">
        <h3>ORTHOGRAPHIC VIEWS</h3>
        <div class="view-grid">
            <div class="view">
                <div class="view-title">FRONT VIEW</div>
                <img src="front_view.png" alt="Front View">
                <p>Scale: 1:10 (approximate)</p>
            </div>
            <div class="view">
                <div class="view-title">TOP VIEW</div>
                <img src="top_view.png" alt="Top View">
                <p>Scale: 1:10 (approximate)</p>
            </div>
            <div class="view">
                <div class="view-title">SIDE VIEW</div>
                <img src="side_view.png" alt="Side View">
                <p>Scale: 1:10 (approximate)</p>
            </div>
            <div class="view">
                <div class="view-title">ISOMETRIC VIEW</div>
                <img src="isometric_view.png" alt="Isometric View">
                <p>3D Reference View</p>
            </div>
        </div>

        <div class="title-block">
            <h4>DRAWING INFORMATION</h4>
            <table class="specs-table">
                <tr>
                    <th>Title</th>
                    <td>Mini Wall Panel Manufacturing System</td>
                </tr>
                <tr>
                    <th>Drawing Number</th>
                    <td>MP-001</td>
                </tr>
                <tr>
                    <th>Revision</th>
                    <td>A</td>
                </tr>
                <tr>
                    <th>Date</th>
                    <td>2025-10-04</td>
                </tr>
                <tr>
                    <th>Scale</th>
                    <td>As Noted</td>
                </tr>
                <tr>
                    <th>Material</th>
                    <td>6061-T6 Aluminum (Frame), Steel (Base Plates)</td>
                </tr>
                <tr>
                    <th>Finish</th>
                    <td>Anodize Clear (Aluminum), Paint (Steel)</td>
                </tr>
            </table>
        </div>

        <div class="notes">
            <h4>MANUFACTURING NOTES</h4>
            <ol>
                <li><strong>Dimensions:</strong> All dimensions in millimeters unless noted</li>
                <li><strong>Tolerance:</strong> ±0.5mm unless otherwise specified</li>
                <li><strong>Frame:</strong> Use 15x15mm T-slot aluminum extrusion (80/20 compatible)</li>
                <li><strong>Fasteners:</strong> M6 socket head cap screws for frame connections</li>
                <li><strong>Robot Base:</strong> 400x400x20mm aluminum plate, holes as shown</li>
                <li><strong>Work Fixture:</strong> 762x508x15mm aluminum plate with locating pins</li>
                <li><strong>Assembly:</strong> Follow safety procedures for Category 3 systems</li>
                <li><strong>Inspection:</strong> Verify all critical dimensions before final assembly</li>
            </ol>
        </div>
    </div>

    <div class="drawing-sheet">
        <h3>COMPONENT SPECIFICATIONS</h3>
        
        <h4>Frame Assembly (1800 x 1200 x 600mm)</h4>
        <table class="specs-table">
            <tr><th>Component</th><th>Quantity</th><th>Description</th><th>Part Number</th></tr>
            <tr><td>Base Extrusion</td><td>4</td><td>15x15mm T-slot, 1800mm length</td><td>1515-1800</td></tr>
            <tr><td>Side Extrusion</td><td>4</td><td>15x15mm T-slot, 1200mm length</td><td>1515-1200</td></tr>
            <tr><td>Vertical Post</td><td>4</td><td>15x15mm T-slot, 600mm length</td><td>1515-600</td></tr>
            <tr><td>Corner Bracket</td><td>8</td><td>90° corner connector</td><td>CB-1515-90</td></tr>
            <tr><td>M6 SHCS</td><td>32</td><td>M6x20mm socket head cap screw</td><td>SHCS-M6-20</td></tr>
        </table>

        <h4>Robot Base Assembly</h4>
        <table class="specs-table">
            <tr><th>Component</th><th>Quantity</th><th>Description</th><th>Material</th></tr>
            <tr><td>Base Plate</td><td>1</td><td>400x400x20mm machined plate</td><td>6061-T6 Aluminum</td></tr>
            <tr><td>Corner Mount</td><td>4</td><td>M10 clearance holes, counterbored</td><td>-</td></tr>
            <tr><td>Motor Mount</td><td>6</td><td>M8 threaded holes, 80mm bolt circle</td><td>-</td></tr>
        </table>

        <h4>Work Fixture Assembly</h4>
        <table class="specs-table">
            <tr><th>Component</th><th>Quantity</th><th>Description</th><th>Material</th></tr>
            <tr><td>Fixture Plate</td><td>1</td><td>762x508x15mm machined plate</td><td>6061-T6 Aluminum</td></tr>
            <tr><td>Locating Pin</td><td>4</td><td>12mm diameter, press fit</td><td>Steel, hardened</td></tr>
            <tr><td>Clamp Assembly</td><td>2</td><td>Pneumatic workholding clamp</td><td>-</td></tr>
        </table>
    </div>

    <script>
        // Print functionality
        window.addEventListener('load', function() {
            console.log('Technical drawings loaded successfully');
        });
        
        // Add print button functionality
        document.addEventListener('keydown', function(e) {
            if (e.ctrlKey && e.key === 'p') {
                e.preventDefault();
                window.print();
            }
        });
    </script>
</body>
</html>
EOF

echo "✓ HTML technical drawing viewer created"

# Method 3: Create a simple PDF using wkhtmltopdf if available
if command -v wkhtmltopdf &> /dev/null; then
    echo "Creating PDF from HTML..."
    wkhtmltopdf --page-size A3 --orientation Portrait \
                --margin-top 20mm --margin-bottom 20mm \
                --margin-left 15mm --margin-right 15mm \
                "$OUTPUT_DIR/technical_drawings.html" \
                "$OUTPUT_DIR/technical_drawings.pdf"
    echo "✓ PDF created: technical_drawings.pdf"
else
    echo "Install wkhtmltopdf for PDF generation: sudo apt install wkhtmltopdf"
fi

# Create summary
echo ""
echo "========================================="
echo "SUCCESS! Technical Drawings Generated"
echo "========================================="
echo ""
echo "Generated Files:"
ls -la "$OUTPUT_DIR" | grep -E '\.(png|html|pdf)$' | awk '{print "  ✓ " $9 " (" $5 " bytes)"}'

echo ""
echo "View Instructions:"
echo "  HTML Viewer: firefox '$OUTPUT_DIR/technical_drawings.html'"
echo "  Images: eog '$OUTPUT_DIR/front_view.png'"
if [ -f "$OUTPUT_DIR/technical_drawings.pdf" ]; then
    echo "  PDF: evince '$OUTPUT_DIR/technical_drawings.pdf'"
fi

echo ""
echo "Print Instructions:"
echo "  1. Open HTML file in browser"
echo "  2. Press Ctrl+P to print"
echo "  3. Select 'Save as PDF' or print to paper"

# Create a quick viewer script
cat > "$OUTPUT_DIR/view_drawings.sh" << EOF
#!/bin/bash
# Quick viewer for technical drawings

echo "Opening technical drawings..."

# Try different viewers in order of preference
if command -v firefox &> /dev/null; then
    firefox '$OUTPUT_DIR/technical_drawings.html' &
elif command -v chromium-browser &> /dev/null; then
    chromium-browser '$OUTPUT_DIR/technical_drawings.html' &
elif command -v google-chrome &> /dev/null; then
    google-chrome '$OUTPUT_DIR/technical_drawings.html' &
else
    echo "No web browser found. Please open technical_drawings.html manually."
fi

# Also open image viewer for individual images
if command -v eog &> /dev/null; then
    eog '$OUTPUT_DIR/isometric_view.png' &
fi

echo "Technical drawings opened in browser and image viewer."
EOF

chmod +x "$OUTPUT_DIR/view_drawings.sh"
echo "✓ Quick viewer script created: view_drawings.sh"

echo ""
echo "Next time, just run: $OUTPUT_DIR/view_drawings.sh"