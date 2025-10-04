# OpenSCAD Model Viewing and Usage Guide
*Complete procedure for viewing and modifying parametric 3D models*

## Quick Start - Viewing OpenSCAD Models

### Step 1: Install OpenSCAD
```bash
# Ubuntu/Debian installation
sudo apt update
sudo apt install openscad

# Verify installation
openscad --version
```

### Step 2: Open Your Model
```bash
# Navigate to CAD directory
cd /home/arm1/APM/Projects/Active/Mini_Prototype/CAD

# Open the mini prototype model
openscad mini_prototype_model.scad
```

### Step 3: View and Render
1. **Preview Mode** (F5 or View → Preview):
   - Fast wireframe rendering
   - Good for quick parameter changes
   - Shows overall geometry

2. **Render Mode** (F6 or View → Render):
   - Full solid rendering with surfaces
   - Required before exporting STL files
   - Takes longer but shows final result

### Step 4: Export Options
```bash
# Export to STL for 3D printing
File → Export → Export as STL...

# Export to other formats
File → Export → Export as AMF/3MF/OFF/DXF...
```

## Parameter Modification Workflow

### Current Model Parameters
The `mini_prototype_model.scad` includes these customizable parameters:

```openscad
// Frame dimensions
frame_width = 1800;      // mm
frame_height = 1200;     // mm
frame_depth = 800;       // mm

// Extrusion profiles
extrusion_size = 15;     // 15x15mm T-slot

// Robot base
robot_base_size = 400;   // 400x400mm plate
robot_base_thickness = 20; // mm

// Work fixture
fixture_width = 762;     // mm
fixture_height = 508;    // mm
fixture_thickness = 15;  // mm

// Panel dimensions
panel_width = 610;       // Standard wall panel
panel_height = 305;      // mm
panel_thickness = 12;    // mm
```

### Modification Process
1. **Open model in OpenSCAD**
2. **Edit parameters** at top of file
3. **Press F5** for preview
4. **Press F6** when satisfied to render
5. **Export** if needed for manufacturing

## Advanced Features

### Customization Console
- **Window → Hide/Show Console** - Shows compilation messages
- **Window → Hide/Show Customizer** - GUI parameter editor
- **View → Animate** - Animation for moving parts

### File Management
```bash
# Create project backup
cp mini_prototype_model.scad mini_prototype_model_$(date +%Y%m%d).scad

# Version control integration
git add *.scad
git commit -m "Updated OpenSCAD model parameters"
```

## Troubleshooting

### Common Issues
1. **Model not appearing**: Check console for syntax errors
2. **Slow rendering**: Reduce `$fn` parameter for circles/curves
3. **Memory issues**: Simplify complex intersections

### Performance Optimization
```openscad
// Low detail for fast preview
$fn = 16;   // Circle resolution

// High detail for final render
$fn = 64;   // Smooth curves
```

## Integration Workflow

### From OpenSCAD to Manufacturing
1. **Design** → OpenSCAD parametric model
2. **Verify** → F6 render and visual check
3. **Export** → STL for 3D printing or DXF for machining
4. **CAM** → Import into Fusion 360/CAM software
5. **Manufacture** → CNC machining or 3D printing

### Model Hierarchy
```
mini_prototype_model.scad
├── Frame Assembly
│   ├── Base frame (1800x1200mm)
│   ├── Vertical supports
│   └── Cross members
├── Robot Base
│   ├── Mounting plate (400x400mm)
│   ├── Motor mounting holes
│   └── Corner fasteners
├── Work Fixture
│   ├── Fixture plate (762x508mm)
│   ├── Locating pins
│   └── Clamp positions
└── Panel Representation
    ├── Panel outline (610x305mm)
    └── Material thickness
```

## Quick Reference Commands

### OpenSCAD Shortcuts
- **F5** - Preview (fast)
- **F6** - Render (final)
- **F4** - Reload and Preview
- **Ctrl+D** - Export Design
- **Ctrl+R** - Reload Design

### Useful Functions
```openscad
// Basic shapes
cube([x, y, z]);
cylinder(h=height, r=radius);
sphere(r=radius);

// Transformations
translate([x, y, z]) object();
rotate([x, y, z]) object();
scale([x, y, z]) object();

// Boolean operations
union() { object1(); object2(); }
difference() { object1(); object2(); }
intersection() { object1(); object2(); }
```

## Model Validation Checklist

### Before Manufacturing
- [ ] All dimensions match engineering drawings
- [ ] Clearances verified (minimum 0.5mm for fits)
- [ ] Material thicknesses realistic
- [ ] Fastener holes properly sized
- [ ] Assembly sequence feasible
- [ ] No overlapping parts in final assembly
- [ ] Export files generated successfully

### Quality Control
- [ ] Visual inspection in render mode
- [ ] Dimension verification with calipers tool
- [ ] Interference checking between components
- [ ] Material volume calculations reasonable
- [ ] Mounting hole patterns verified

---

**Remember**: OpenSCAD is parametric - change numbers at the top, get new geometry instantly!

**Emergency Contact**: If model won't open, check file permissions and OpenSCAD installation.