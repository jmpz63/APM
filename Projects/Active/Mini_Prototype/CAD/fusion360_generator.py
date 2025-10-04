# Fusion 360 Python API Script - Mini Prototype Generator
# Place this file in Fusion 360 Scripts folder and run from Scripts and Add-Ins panel

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import math

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        # Create new document
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = app.activeProduct
        
        # Get root component
        rootComp = design.rootComponent
        rootComp.name = "Mini Wall Panel Manufacturing System"
        
        # Create main assembly
        create_frame_assembly(rootComp)
        create_robot_base(rootComp)
        create_work_fixture(rootComp)
        create_material_handling(rootComp)
        
        # Fit view to show entire model
        app.activeViewport.fit()
        
        ui.messageBox('Mini prototype model created successfully!')
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def create_frame_assembly(rootComp):
    """Create the main frame using 80/20 T-slot extrusion"""
    
    # Create component for frame
    frame_occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    frame_comp = frame_occ.component
    frame_comp.name = "Frame Assembly"
    
    # Parameters
    frame_width = 180.0  # 1800mm in cm for Fusion 360
    frame_height = 120.0  # 1200mm
    frame_depth = 60.0   # 600mm
    extrusion_size = 1.5  # 15mm
    
    # Create base frame rectangle
    sketches = frame_comp.sketches
    xyPlane = frame_comp.xYConstructionPlane
    sketch = sketches.add(xyPlane)
    lines = sketch.sketchCurves.sketchLines
    
    # Draw frame outline
    rect_lines = []
    rect_lines.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(frame_width, 0, 0)
    ))
    rect_lines.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(frame_width, 0, 0),
        adsk.core.Point3D.create(frame_width, frame_height, 0)
    ))
    rect_lines.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(frame_width, frame_height, 0),
        adsk.core.Point3D.create(0, frame_height, 0)
    ))
    rect_lines.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, frame_height, 0),
        adsk.core.Point3D.create(0, 0, 0)
    ))
    
    # Add center cross members
    lines.addByTwoPoints(
        adsk.core.Point3D.create(0, frame_height/2, 0),
        adsk.core.Point3D.create(frame_width, frame_height/2, 0)
    )
    lines.addByTwoPoints(
        adsk.core.Point3D.create(frame_width/2, 0, 0),
        adsk.core.Point3D.create(frame_width/2, frame_height, 0)
    )
    
    # Create T-slot profile for extrusion
    create_tslot_profile(frame_comp, extrusion_size)
    
    return frame_comp

def create_tslot_profile(component, size):
    """Create 15x15mm T-slot extrusion profile"""
    
    # Create sketch for T-slot profile
    sketches = component.sketches
    yzPlane = component.yZConstructionPlane
    profile_sketch = sketches.add(yzPlane)
    lines = profile_sketch.sketchCurves.sketchLines
    
    half_size = size / 2.0
    
    # Outer square
    outer_rect = []
    outer_rect.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, -half_size, -half_size),
        adsk.core.Point3D.create(0, half_size, -half_size)
    ))
    outer_rect.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, half_size, -half_size),
        adsk.core.Point3D.create(0, half_size, half_size)
    ))
    outer_rect.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, half_size, half_size),
        adsk.core.Point3D.create(0, -half_size, half_size)
    ))
    outer_rect.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, -half_size, half_size),
        adsk.core.Point3D.create(0, -half_size, -half_size)
    ))
    
    # T-slot groove (6mm wide x 3mm deep)
    groove_width = 0.6  # 6mm
    groove_depth = 0.3  # 3mm
    
    # Top T-groove
    t_groove = []
    t_groove.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, -groove_width/2, half_size),
        adsk.core.Point3D.create(0, groove_width/2, half_size)
    ))
    t_groove.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, groove_width/2, half_size),
        adsk.core.Point3D.create(0, groove_width/2, half_size - groove_depth)
    ))
    t_groove.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, groove_width/2, half_size - groove_depth),
        adsk.core.Point3D.create(0, -groove_width/2, half_size - groove_depth)
    ))
    t_groove.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, -groove_width/2, half_size - groove_depth),
        adsk.core.Point3D.create(0, -groove_width/2, half_size)
    ))
    
    return profile_sketch

def create_robot_base(rootComp):
    """Create robot base plate with mounting holes"""
    
    # Create component for robot base
    base_occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    base_comp = base_occ.component
    base_comp.name = "Robot Base Assembly"
    
    # Position at center of frame
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(90, 30, 0)  # Center position
    base_occ.transform = transform
    
    # Create base plate sketch
    sketches = base_comp.sketches
    xyPlane = base_comp.xYConstructionPlane
    sketch = sketches.add(xyPlane)
    
    # Draw 400x400mm base plate
    plate_size = 40.0  # 400mm
    rect = sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(-plate_size/2, -plate_size/2, 0),
        adsk.core.Point3D.create(plate_size/2, plate_size/2, 0)
    )
    
    # Add corner radius
    corner_radius = 0.5  # 5mm
    for line in rect:
        sketch.sketchCurves.sketchArcs.addFillet(line, line, corner_radius)
    
    # Extrude base plate
    prof = sketch.profiles.item(0)
    extrudes = base_comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2.0))  # 20mm thick
    base_ext = extrudes.add(extInput)
    
    # Add mounting holes
    add_mounting_holes(base_comp, base_ext.bodies.item(0))
    
    return base_comp

def add_mounting_holes(component, body):
    """Add mounting holes to robot base plate"""
    
    # Get top face of base plate
    top_face = None
    for face in body.faces:
        if face.geometry.normal.z > 0.9:  # Top face
            top_face = face
            break
    
    if not top_face:
        return
    
    # Create sketch on top face
    sketches = component.sketches
    hole_sketch = sketches.add(top_face)
    circles = hole_sketch.sketchCurves.sketchCircles
    
    # Corner mounting holes (M10 clearance, 10.5mm dia)
    hole_positions = [
        (-15, -15), (15, -15), (15, 15), (-15, 15)
    ]
    
    for pos in hole_positions:
        circles.addByCenterRadius(
            adsk.core.Point3D.create(pos[0], pos[1], 0),
            0.525  # 10.5mm diameter
        )
    
    # Motor mounting holes (M8 threaded, 6x on 80mm bolt circle)
    bolt_circle_radius = 4.0  # 80mm diameter / 2
    thread_radius = 0.34  # 6.8mm tap drill diameter
    
    for i in range(6):
        angle = i * 60 * math.pi / 180  # 60 degrees between holes
        x = bolt_circle_radius * math.cos(angle)
        y = bolt_circle_radius * math.sin(angle)
        
        circles.addByCenterRadius(
            adsk.core.Point3D.create(x, y, 0),
            thread_radius
        )
    
    # Cut holes
    holes = component.features.holeFeatures
    for circle in circles:
        hole_input = holes.createSimpleInput(adsk.core.ValueInput.createByReal(circle.radius * 2))
        hole_input.setPositionBySketchPoint(circle.centerSketchPoint)
        holes.add(hole_input)

def create_work_fixture(rootComp):
    """Create work fixture for panel assembly"""
    
    # Create component for work fixture
    fixture_occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    fixture_comp = fixture_occ.component
    fixture_comp.name = "Work Fixture"
    
    # Position fixture
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(90, 80, 0)
    fixture_occ.transform = transform
    
    # Create fixture plate (762mm x 508mm x 20mm)
    sketches = fixture_comp.sketches
    xyPlane = fixture_comp.xYConstructionPlane
    sketch = sketches.add(xyPlane)
    
    # Draw fixture plate outline
    fixture_width = 76.2   # 762mm
    fixture_height = 50.8  # 508mm
    
    rect = sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(-fixture_width/2, -fixture_height/2, 0),
        adsk.core.Point3D.create(fixture_width/2, fixture_height/2, 0)
    )
    
    # Extrude fixture plate
    prof = sketch.profiles.item(0)
    extrudes = fixture_comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2.0))  # 20mm thick
    fixture_ext = extrudes.add(extInput)
    
    # Add panel area recess
    add_panel_recess(fixture_comp, fixture_ext.bodies.item(0))
    
    return fixture_comp

def add_panel_recess(component, body):
    """Add recessed area for 24" x 12" panel"""
    
    # Get top face
    top_face = None
    for face in body.faces:
        if face.geometry.normal.z > 0.9:
            top_face = face
            break
    
    if not top_face:
        return
    
    # Create sketch for panel recess
    sketches = component.sketches
    recess_sketch = sketches.add(top_face)
    
    # 24" x 12" panel area (610mm x 305mm)
    panel_width = 61.0   # 610mm
    panel_height = 30.5  # 305mm
    
    rect = recess_sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(-panel_width/2, -panel_height/2, 0),
        adsk.core.Point3D.create(panel_width/2, panel_height/2, 0)
    )
    
    # Cut recess (1mm deep)
    prof = recess_sketch.profiles.item(0)
    extrudes = component.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(0.1))  # 1mm deep
    recess_ext = extrudes.add(extInput)

def create_material_handling(rootComp):
    """Create material feed system"""
    
    # Create component for material handling
    material_occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    material_comp = material_occ.component
    material_comp.name = "Material Handling"
    
    # Position material feed
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(160, 60, 20)
    material_occ.transform = transform
    
    # Create feed chute sketch
    sketches = material_comp.sketches
    xyPlane = material_comp.xYConstructionPlane
    sketch = sketches.add(xyPlane)
    
    # Draw chute profile (trapezoidal)
    lines = sketch.sketchCurves.sketchLines
    chute_profile = []
    chute_profile.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(10, 0, 0)
    ))
    chute_profile.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(10, 0, 0),
        adsk.core.Point3D.create(8, 8, 0)
    ))
    chute_profile.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(8, 8, 0),
        adsk.core.Point3D.create(2, 8, 0)
    ))
    chute_profile.append(lines.addByTwoPoints(
        adsk.core.Point3D.create(2, 8, 0),
        adsk.core.Point3D.create(0, 0, 0)
    ))
    
    # Extrude chute
    prof = sketch.profiles.item(0)
    extrudes = material_comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(60.0))  # 600mm long
    chute_ext = extrudes.add(extInput)
    
    return material_comp

if __name__ == '__main__':
    run(None)