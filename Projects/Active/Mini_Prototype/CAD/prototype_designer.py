#!/usr/bin/env python3
"""
Mini Wall Panel Manufacturing Prototype - CAD Design
Complete 3D model for scaled-down demonstration system
Engineering specifications and bill of materials
"""

import math
import json
from datetime import datetime

class MiniPrototypeDesigner:
    def __init__(self):
        self.scale = 0.25  # 1:4 scale
        self.base_width = 72  # 6 feet
        self.base_depth = 48  # 4 feet  
        self.work_height = 30 # 30 inches working height
```

**PRIORITY 2: Workspace Setup (1 hour)**
- [ ] **Clear 4'x6' workspace area** for prototype assembly
- [ ] **Tool inventory** - ensure basic hand tools available
- [ ] **Power requirements** - verify 220V available for equipment
- [ ] **Safety setup** - establish safe work practices

**PRIORITY 3: CAD Design Finalization (2 hours)**

<function_calls>
<invoke name="create_file">
<parameter name="content">#!/usr/bin/env python3
"""
Mini Wall Panel Manufacturing Prototype - CAD Design
Complete 3D model for scaled-down demonstration system
"""

import cadquery as cq
from math import pi, cos, sin

class MiniPrototypeDesigner:
    def __init__(self):
        self.scale = 0.25  # 1:4 scale
        self.base_width = 72  # 6 feet
        self.base_depth = 48  # 4 feet  
        self.work_height = 30 # 30 inches working height
        
    def create_main_frame(self):
        """Design the main aluminum extrusion frame"""
        
        # Base platform using 80/20 extrusion
        base_frame = (cq.Workplane("XY")
                     .rect(self.base_width, self.base_depth)
                     .extrude(3)  # 3" thick base
                     .edges("|Z").fillet(0.5))
        
        # Vertical supports (4 corners)
        corner_posts = (cq.Workplane("XY")
                       .rect(self.base_width-6, self.base_depth-6, forConstruction=True)  
                       .vertices()
                       .rect(3, 3)
                       .extrude(36))  # 36" height
        
        # Cross bracing for rigidity
        cross_brace_x = (cq.Workplane("XZ", origin=(0, self.base_depth/2-3, 18))
                        .rect(self.base_width-6, 3)
                        .extrude(1.5))
        
        cross_brace_y = (cq.Workplane("YZ", origin=(self.base_width/2-3, 0, 18))
                        .rect(self.base_depth-6, 3)  
                        .extrude(1.5))
        
        return base_frame.union(corner_posts).union(cross_brace_x).union(cross_brace_y)
    
    def create_robot_mount(self):
        """Design the robot mounting platform"""
        
        # Robot base platform
        robot_platform = (cq.Workplane("XY", origin=(12, 0, 33))
                         .rect(18, 18)
                         .extrude(2)
                         .faces(">Z").workplane()
                         .hole(8))  # Central mounting hole for robot base
        
        # Reinforcement ribs
        ribs = (cq.Workplane("XY", origin=(12, 0, 34))
               .rect(16, 2, forConstruction=True)
               .vertices()
               .rect(2, 14)
               .extrude(1))
        
        return robot_platform.union(ribs)
    
    def create_material_feed_system(self):
        """Design the automated lumber feeding mechanism"""
        
        # Feed chute for lumber pieces
        chute_angle = 15  # degrees from horizontal
        chute_length = 30
        
        # Main feed rails
        feed_rails = (cq.Workplane("XY", origin=(-24, 0, 24))
                     .transformed(rotate=(0, -chute_angle, 0))
                     .rect(chute_length, 2)
                     .extrude(1)
                     .translate((0, 8, 0))
                     .union(
                         cq.Workplane("XY", origin=(-24, 0, 24))
                         .transformed(rotate=(0, -chute_angle, 0))
                         .rect(chute_length, 2)
                         .extrude(1)
                         .translate((0, -8, 0))
                     ))
        
        # Pneumatic pusher mechanism
        pusher_cylinder = (cq.Workplane("XZ", origin=(-30, 0, 28))
                          .circle(1.5)
                          .extrude(8))  # 8" stroke pneumatic cylinder
        
        pusher_rod = (cq.Workplane("XZ", origin=(-22, 0, 28))
                     .circle(0.5)
                     .extrude(12))  # Pusher rod extension
        
        return feed_rails.union(pusher_cylinder).union(pusher_rod)
    
    def create_assembly_jig(self):
        """Design precision assembly fixtures for mini panels"""
        
        # Main assembly work surface
        work_surface = (cq.Workplane("XY", origin=(0, 0, self.work_height))
                       .rect(30, 20)
                       .extrude(1))
        
        # Positioning stops for 24" x 12" mini panels
        panel_width = 24 * self.scale  # 6" actual
        panel_height = 12 * self.scale  # 3" actual  
        
        # Corner positioning blocks
        corner_stops = (cq.Workplane("XY", origin=(0, 0, self.work_height + 1))
                       .rect(panel_width + 2, panel_height + 2, forConstruction=True)
                       .vertices()
                       .rect(1, 1)
                       .extrude(2))
        
        # Clamping mechanism
        clamps = (cq.Workplane("XY", origin=(0, 0, self.work_height + 1))
                 .rect(panel_width - 2, panel_height - 2, forConstruction=True)
                 .vertices()
                 .circle(2)
                 .extrude(0.5))  # Pneumatic clamps
        
        return work_surface.union(corner_stops).union(clamps)
    
    def create_nail_gun_system(self):
        """Design the automated nail gun positioning system"""
        
        # Linear actuator for nail gun positioning
        actuator_rail = (cq.Workplane("XY", origin=(0, 15, 40))
                        .rect(24, 2)
                        .extrude(1))
        
        # Nail gun carriage
        carriage = (cq.Workplane("XY", origin=(0, 15, 41))
                   .rect(4, 6)
                   .extrude(3)
                   .faces(">Z").workplane()
                   .rect(2, 8)
                   .cutThruAll())  # Slot for nail gun mount
        
        # Pneumatic nail gun (simplified representation)
        nail_gun = (cq.Workplane("XY", origin=(0, 15, 44))
                   .rect(2, 8)
                   .extrude(6)
                   .faces(">Z").workplane()
                   .circle(0.5)
                   .extrude(2))  # Nail gun tip
        
        return actuator_rail.union(carriage).union(nail_gun)
    
    def create_quality_inspection_station(self):
        """Design vision-based quality control system"""
        
        # Camera mounting arm
        camera_arm = (cq.Workplane("XY", origin=(18, 0, 50))
                     .rect(2, 2)
                     .extrude(8)
                     .faces(">Z").workplane()
                     .transformed(rotate=(45, 0, 0))
                     .rect(2, 12)
                     .extrude(1))  # Angled camera mount
        
        # Camera housing
        camera = (cq.Workplane("XY", origin=(18, 6, 56))
                 .circle(2)
                 .extrude(4))
        
        # LED lighting ring
        led_ring = (cq.Workplane("XY", origin=(18, 6, 52))
                   .circle(4)
                   .circle(3)
                   .extrude(0.5))
        
        return camera_arm.union(camera).union(led_ring)
    
    def create_safety_systems(self):
        """Design safety features and emergency stops"""
        
        # Emergency stop posts at corners
        estop_posts = (cq.Workplane("XY")
                      .rect(self.base_width + 6, self.base_depth + 6, forConstruction=True)
                      .vertices()
                      .circle(1)
                      .extrude(48))  # 4' high safety posts
        
        # Safety light tower
        light_tower = (cq.Workplane("XY", origin=(self.base_width/2 + 6, 0, 48))
                      .circle(2)
                      .extrude(12)
                      .faces(">Z").workplane()
                      .rect(4, 4)
                      .extrude(1))  # Status light housing
        
        # Perimeter safety sensors
        sensors = (cq.Workplane("XY", origin=(0, 0, 24))
                  .rect(self.base_width + 12, self.base_depth + 12, forConstruction=True)
                  .vertices()
                  .circle(0.5)
                  .extrude(2))
        
        return estop_posts.union(light_tower).union(sensors)
    
    def create_complete_prototype(self):
        """Assemble all components into complete system"""
        
        # Create all subsystems
        main_frame = self.create_main_frame()
        robot_mount = self.create_robot_mount()  
        feed_system = self.create_material_feed_system()
        assembly_jig = self.create_assembly_jig()
        nail_gun_system = self.create_nail_gun_system()
        quality_station = self.create_quality_inspection_station()
        safety_systems = self.create_safety_systems()
        
        # Combine all components
        complete_system = (main_frame
                          .union(robot_mount)
                          .union(feed_system) 
                          .union(assembly_jig)
                          .union(nail_gun_system)
                          .union(quality_station)
                          .union(safety_systems))
        
        return complete_system
    
    def generate_manufacturing_files(self):
        """Generate files for manufacturing and assembly"""
        
        prototype = self.create_complete_prototype()
        
        # Export complete assembly
        prototype.exportStep("/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/complete_prototype.step")
        prototype.exportStl("/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/complete_prototype.stl")
        
        # Export individual components for manufacturing
        components = {
            "main_frame": self.create_main_frame(),
            "robot_mount": self.create_robot_mount(),
            "feed_system": self.create_material_feed_system(),
            "assembly_jig": self.create_assembly_jig(),
            "nail_gun_system": self.create_nail_gun_system()
        }
        
        for name, component in components.items():
            component.exportStep(f"/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/{name}.step")
            component.exportStl(f"/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/{name}.stl")
        
        print("✅ All CAD files generated successfully!")
        print("📁 Files saved to: /home/arm1/APM/Projects/Active/Mini_Prototype/CAD/")
        
        # Generate bill of materials
        self.generate_bom()
        
    def generate_bom(self):
        """Generate detailed bill of materials"""
        
        bom = {
            "Frame Components": {
                "80/20 T-Slot Extrusion 1515 x 72": {"qty": 4, "cost": 25.00, "supplier": "8020.net"},
                "80/20 T-Slot Extrusion 1515 x 48": {"qty": 6, "cost": 18.00, "supplier": "8020.net"},  
                "80/20 T-Slot Extrusion 1515 x 36": {"qty": 4, "cost": 15.00, "supplier": "8020.net"},
                "Corner Brackets 15-series": {"qty": 12, "cost": 8.50, "supplier": "8020.net"},
                "T-Nuts and Bolts Kit": {"qty": 1, "cost": 45.00, "supplier": "8020.net"}
            },
            "Motion Components": {
                "NEMA 23 Stepper Motor 425oz-in": {"qty": 6, "cost": 45.00, "supplier": "StepperOnline"},
                "DM556 Stepper Driver": {"qty": 6, "cost": 25.00, "supplier": "StepperOnline"},
                "48V 12A Power Supply": {"qty": 2, "cost": 65.00, "supplier": "Amazon"},
                "Linear Rail MGN15 300mm": {"qty": 2, "cost": 35.00, "supplier": "Amazon"},
                "Lead Screw 8mm x 300mm": {"qty": 2, "cost": 28.00, "supplier": "Amazon"}
            },
            "Pneumatic Components": {
                "6-Gallon Air Compressor": {"qty": 1, "cost": 89.00, "supplier": "Harbor Freight"},
                "Pneumatic Cylinder 2x8": {"qty": 3, "cost": 25.00, "supplier": "Fastenal"},
                "5/2 Solenoid Valve": {"qty": 3, "cost": 18.00, "supplier": "Fastenal"},
                "Air Fittings Kit": {"qty": 1, "cost": 35.00, "supplier": "Fastenal"},
                "Pneumatic Tubing 1/4": {"qty": 50, "cost": 25.00, "supplier": "Fastenal"}
            },
            "Electronics & Sensors": {
                "USB Camera 1080p": {"qty": 2, "cost": 45.00, "supplier": "Amazon"},
                "LED Ring Light": {"qty": 2, "cost": 25.00, "supplier": "Amazon"},
                "Limit Switches": {"qty": 8, "cost": 5.00, "supplier": "Amazon"},
                "Emergency Stop Button": {"qty": 4, "cost": 12.00, "supplier": "Amazon"},
                "24V LED Strip Lights": {"qty": 2, "cost": 15.00, "supplier": "Amazon"}
            },
            "Tools & Fixtures": {
                "Pneumatic Brad Nailer": {"qty": 1, "cost": 85.00, "supplier": "Home Depot"},
                "Aluminum Angle Stock": {"qty": 10, "cost": 8.00, "supplier": "Home Depot"},
                "Clamps and Fixtures": {"qty": 8, "cost": 15.00, "supplier": "Home Depot"},
                "Assembly Hardware": {"qty": 1, "cost": 50.00, "supplier": "Fastenal"}
            }
        }
        
        # Calculate totals
        total_cost = 0
        bom_file_content = "# Mini Prototype Bill of Materials\n\n"
        
        for category, items in bom.items():
            bom_file_content += f"## {category}\n\n"
            category_total = 0
            
            for item, details in items.items():
                item_total = details["qty"] * details["cost"]
                category_total += item_total
                total_cost += item_total
                
                bom_file_content += f"- **{item}**\n"
                bom_file_content += f"  - Qty: {details['qty']}\n" 
                bom_file_content += f"  - Unit Cost: ${details['cost']:.2f}\n"
                bom_file_content += f"  - Total: ${item_total:.2f}\n"
                bom_file_content += f"  - Supplier: {details['supplier']}\n\n"
            
            bom_file_content += f"**{category} Subtotal: ${category_total:.2f}**\n\n"
        
        bom_file_content += f"## TOTAL PROJECT COST: ${total_cost:.2f}\n\n"
        bom_file_content += "**Budget Status**: "
        if total_cost <= 3500:
            bom_file_content += f"✅ UNDER BUDGET (${3500 - total_cost:.2f} remaining)\n"
        else:
            bom_file_content += f"⚠️ OVER BUDGET (${total_cost - 3500:.2f} excess)\n"
        
        # Save BOM file
        with open("/home/arm1/APM/Projects/Active/Mini_Prototype/Bill_of_Materials.md", "w") as f:
            f.write(bom_file_content)
        
        print(f"💰 Total project cost: ${total_cost:.2f}")
        print("📋 Bill of Materials saved to: Bill_of_Materials.md")

# Generate all prototype design files
if __name__ == "__main__":
    designer = MiniPrototypeDesigner()
    designer.generate_manufacturing_files()
    print("🚀 Mini prototype design complete - ready for manufacturing!")