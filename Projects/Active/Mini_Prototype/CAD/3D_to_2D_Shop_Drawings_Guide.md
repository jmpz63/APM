# 3D to 2D Shop Drawings Workflow
*Complete guide for generating professional shop drawings from OpenSCAD models*

## Method 1: OpenSCAD → FreeCAD TechDraw (Recommended)

### Step 1: Export 3D Model from OpenSCAD
```bash
# Open your OpenSCAD model
openscad /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad

# Export as STEP file (better than STL for engineering)
File → Export → Export as CSG...
# Or export as STL if STEP not available
File → Export → Export as STL...
```

### Step 2: Install FreeCAD
```bash
# Install FreeCAD for technical drawings
sudo apt update
sudo apt install freecad

# Verify installation
freecad --version
```

### Step 3: Create Technical Drawings in FreeCAD

#### Import 3D Model
1. Open FreeCAD
2. File → Import → Select your exported file
3. Switch to **Part Design** workbench
4. Your 3D model appears in the tree

#### Generate 2D Views
1. Switch to **TechDraw** workbench
2. Insert → New Page → Select template (A3, A4, etc.)
3. Insert → New View → Select your 3D part
4. Position the view on the drawing

#### Add Multiple Views
```python
# FreeCAD Python console commands for automated view creation
import TechDraw

# Create front view
front_view = TechDraw.makeViewPart()
front_view.Source = App.ActiveDocument.getObject('YourPartName')
front_view.Direction = (0, -1, 0)  # Front direction

# Create top view  
top_view = TechDraw.makeViewPart()
top_view.Source = App.ActiveDocument.getObject('YourPartName')
top_view.Direction = (0, 0, -1)  # Top direction

# Create side view
side_view = TechDraw.makeViewPart()
side_view.Source = App.ActiveDocument.getObject('YourPartName')
side_view.Direction = (-1, 0, 0)  # Side direction
```

#### Add Dimensions
1. Select **TechDraw** → **Dimension** tools
2. Click on edges/vertices to dimension
3. Dimensions automatically calculate from 3D model

## Method 2: OpenSCAD Direct 2D Export

### Modify OpenSCAD Model for 2D Views
```openscad
// Add this to your mini_prototype_model.scad
// 2D projection modes for shop drawings

// Control what to show
show_3d = true;        // Set false for 2D only
show_front_view = true;
show_top_view = true;
show_side_view = true;

if (show_3d) {
    // Your existing 3D model code here
    complete_assembly();
}

// 2D Projections for shop drawings
if (show_front_view) {
    translate([2000, 0, 0]) {  // Offset to side
        projection(cut=false) {
            rotate([0, 0, 0])
                complete_assembly();
        }
    }
}

if (show_top_view) {
    translate([0, 2000, 0]) {  // Offset above
        projection(cut=false) {
            rotate([90, 0, 0])
                complete_assembly();
        }
    }
}

if (show_side_view) {
    translate([2000, 2000, 0]) {  // Offset diagonal
        projection(cut=false) {
            rotate([0, 90, 0])
                complete_assembly();
        }
    }
}
```

### Export 2D Views
1. Set `show_3d = false` in OpenSCAD
2. Set individual views to `true`
3. Export as DXF: File → Export → Export as DXF
4. Import DXF into LibreCAD for dimensioning

## Method 3: Automated Python Script

### FreeCAD Python Automation
```python
# save as: generate_shop_drawings.py
import FreeCAD
import Import
import TechDraw
import os

def create_shop_drawings(stl_file, output_dir):
    """Generate complete shop drawing set from STL file"""
    
    # Create new document
    doc = FreeCAD.newDocument("ShopDrawings")
    
    # Import 3D model
    Import.insert(stl_file, doc.Name)
    part = doc.getObject(doc.Objects[0].Name)
    
    # Create drawing page
    page = doc.addObject('TechDraw::DrawPage', 'DrawingPage')
    template = doc.addObject('TechDraw::DrawSVGTemplate', 'Template')
    template.Template = '/usr/share/freecad/Mod/TechDraw/Templates/A3_Landscape.svg'
    page.Template = template
    
    # Front view
    front_view = doc.addObject('TechDraw::DrawViewPart', 'FrontView')
    front_view.Source = [part]
    front_view.Direction = (0, -1, 0)
    front_view.X = 100
    front_view.Y = 200
    front_view.Scale = 0.5
    page.addView(front_view)
    
    # Top view
    top_view = doc.addObject('TechDraw::DrawViewPart', 'TopView')
    top_view.Source = [part]
    top_view.Direction = (0, 0, -1)
    top_view.X = 300
    top_view.Y = 200
    top_view.Scale = 0.5
    page.addView(top_view)
    
    # Side view
    side_view = doc.addObject('TechDraw::DrawViewPart', 'SideView')
    side_view.Source = [part]
    side_view.Direction = (-1, 0, 0)
    side_view.X = 100
    side_view.Y = 50
    side_view.Scale = 0.5
    page.addView(side_view)
    
    # Add dimensions (automated)
    add_automatic_dimensions(front_view)
    add_automatic_dimensions(top_view)
    add_automatic_dimensions(side_view)
    
    # Export as PDF
    output_file = os.path.join(output_dir, "shop_drawings.pdf")
    page.exportPDF(output_file)
    
    print(f"Shop drawings exported to: {output_file}")
    
    return doc

def add_automatic_dimensions(view):
    """Add automatic dimensions to a view"""
    doc = view.Document
    
    # Get view edges for dimensioning
    edges = view.getVisibleEdges()
    
    # Add horizontal dimensions
    for i, edge in enumerate(edges[:3]):  # Limit to avoid clutter
        dim = doc.addObject('TechDraw::DrawViewDimension', f'Dimension_{i}')
        dim.References2D = [(view, [edge])]
        dim.MeasureType = 'Distance'
        view.addDimension(dim)

# Usage
if __name__ == "__main__":
    stl_file = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.stl"
    output_dir = "/home/arm1/APM/Projects/Active/Mini_Prototype/Drawings"
    
    create_shop_drawings(stl_file, output_dir)
```

## Method 4: LibreCAD for Final Drawings

### Install LibreCAD
```bash
sudo apt install librecad
```

### Workflow
1. Import DXF from OpenSCAD
2. Add dimensions manually
3. Add annotations and title blocks
4. Export as PDF

## Complete Automated Workflow Script

### Master Script for Shop Drawings
```bash
#!/bin/bash
# Complete 3D to 2D shop drawing generation

echo "Generating shop drawings from 3D model..."

# Step 1: Export from OpenSCAD
echo "Exporting 3D model to STL..."
openscad -o /tmp/model.stl /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad

# Step 2: Generate 2D projections
echo "Creating 2D projections..."
openscad -D 'show_3d=false' -D 'show_front_view=true' -o /tmp/front_view.dxf /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad

openscad -D 'show_3d=false' -D 'show_top_view=true' -o /tmp/top_view.dxf /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad

openscad -D 'show_3d=false' -D 'show_side_view=true' -o /tmp/side_view.dxf /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/mini_prototype_model.scad

# Step 3: Create FreeCAD drawings
echo "Generating technical drawings..."
freecad --run-python /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/generate_shop_drawings.py

echo "Shop drawings complete!"
echo "Check: /home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/"
```

## Professional Drawing Standards

### Title Block Template
```
┌─────────────────────────────────────────────────────┐
│ MINI WALL PANEL MANUFACTURING SYSTEM               │
│ ROBOT BASE ASSEMBLY                                 │
├─────────────────────────────────────────────────────┤
│ SCALE: 1:4        DATE: 2025-10-04    DWG: RB-001  │
│ MATERIAL: 6061-T6 ALUMINUM PLATE                   │
│ FINISH: ANODIZE CLEAR                               │
├─────────────────────────────────────────────────────┤
│ DRAWN: APM ENG    CHECKED: QA    APPROVED: MGR     │
└─────────────────────────────────────────────────────┘
```

### Dimension Standards
- All dimensions in millimeters
- Tolerance: ±0.5mm unless noted
- Hole dimensions: +0.1/0.0mm
- Critical dimensions: ±0.1mm
- Angular tolerance: ±1°

### View Layout
```
┌─────────────────────────────────────┐
│  FRONT VIEW        TOP VIEW         │
│                                     │
│     ┌─────┐         ┌─────┐         │
│     │     │         │     │         │
│     │     │         │     │         │
│     └─────┘         └─────┘         │
│                                     │
│  SIDE VIEW         DETAIL A         │
│                                     │
│     ┌─────┐         ┌─────┐         │
│     │     │         │  ○  │         │
│     │     │         │     │         │
│     └─────┘         └─────┘         │
└─────────────────────────────────────┘
```

## Quality Control Checklist

### Before Release
- [ ] All critical dimensions shown
- [ ] Material specifications included
- [ ] Tolerance callouts complete
- [ ] Hole sizes and locations verified
- [ ] Assembly references clear
- [ ] Title block information complete
- [ ] Drawing number assigned
- [ ] Revision control implemented
- [ ] PDF export successful
- [ ] Drawing legibility verified

## Output Formats

### Recommended Export Options
1. **PDF** - Universal, preserves precision
2. **DXF** - CAD interchange format
3. **SVG** - Web viewing, scalable
4. **PNG** - Quick viewing (300 DPI minimum)

---

**Result**: Professional shop drawings automatically generated from your 3D OpenSCAD models using free, open-source tools!