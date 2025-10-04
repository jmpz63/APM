// OpenSCAD Model - Complete Mini Prototype Assembly
// Parametric design for wall panel manufacturing system

// Global Parameters
frame_width = 1800;        // mm
frame_height = 1200;       // mm
frame_depth = 600;         // mm
extrusion_size = 15;       // mm (1515 series)
base_plate_thickness = 20; // mm
robot_height = 800;        // mm

// Material colors for visualization
aluminum_color = [0.8, 0.8, 0.9, 0.8];
steel_color = [0.5, 0.5, 0.6, 0.9];
plastic_color = [0.2, 0.2, 0.8, 0.7];

// Main assembly
module mini_prototype_assembly() {
    // Frame structure
    color(aluminum_color) frame_structure();
    
    // Robot base and arm
    color(steel_color) {
        translate([900, 300, 0]) robot_base_assembly();
        translate([900, 300, base_plate_thickness]) robot_arm_simplified();
    }
    
    // Work fixture
    color(aluminum_color) translate([900, 800, 0]) work_fixture();
    
    // Material feed system
    color(plastic_color) translate([1600, 600, 200]) material_feed_chute();
    
    // Electrical enclosures
    color([0.3, 0.3, 0.3, 0.9]) {
        translate([200, 200, 0]) electrical_enclosure(300, 400, 200);
        translate([200, 900, 0]) pneumatic_components();
    }
}

// Frame structure using 80/20 T-slot
module frame_structure() {
    // Base frame
    translate([0, 0, 0]) frame_rectangle(frame_width, frame_height);
    translate([0, 0, frame_depth]) frame_rectangle(frame_width, frame_height);
    
    // Vertical posts
    translate([0, 0, 0]) vertical_post(frame_depth);
    translate([frame_width-extrusion_size, 0, 0]) vertical_post(frame_depth);
    translate([0, frame_height-extrusion_size, 0]) vertical_post(frame_depth);
    translate([frame_width-extrusion_size, frame_height-extrusion_size, 0]) vertical_post(frame_depth);
    
    // Cross members for rigidity
    translate([0, frame_height/2 - extrusion_size/2, 0]) 
        tslot_extrusion(frame_width, extrusion_size, extrusion_size);
    translate([frame_width/2 - extrusion_size/2, 0, 0])
        rotate([0, 0, 90]) tslot_extrusion(frame_height, extrusion_size, extrusion_size);
}

module frame_rectangle(width, height) {
    // Bottom rail
    tslot_extrusion(width, extrusion_size, extrusion_size);
    
    // Top rail
    translate([0, height-extrusion_size, 0])
        tslot_extrusion(width, extrusion_size, extrusion_size);
    
    // Left rail
    rotate([0, 0, 90])
        tslot_extrusion(height, extrusion_size, extrusion_size);
    
    // Right rail
    translate([width-extrusion_size, 0, 0])
        rotate([0, 0, 90])
        tslot_extrusion(height, extrusion_size, extrusion_size);
}

module vertical_post(height) {
    rotate([90, 0, 0])
        translate([0, 0, -extrusion_size])
        tslot_extrusion(height, extrusion_size, extrusion_size);
}

// 1515 T-slot extrusion profile
module tslot_extrusion(length, width, height) {
    difference() {
        cube([length, width, height]);
        
        // T-slot grooves on all 4 sides
        translate([0, width/2, height-2]) 
            cube([length, 6, 3]);
        translate([0, width/2-3, height-5]) 
            cube([length, 6, 2]);
            
        translate([0, 2, width/2]) 
            rotate([0, 90, 0]) cube([6, 3, length]);
        translate([0, -1, width/2-3]) 
            rotate([0, 90, 0]) cube([6, 2, length]);
            
        translate([0, width/2, 2]) 
            cube([length, 6, 3]);
        translate([0, width/2-3, 0]) 
            cube([length, 6, 2]);
            
        translate([0, width-2, width/2]) 
            rotate([0, 90, 0]) cube([6, 3, length]);
        translate([0, width+1, width/2-3]) 
            rotate([0, 90, 0]) cube([6, 2, length]);
    }
}

// Robot base plate
module robot_base_assembly() {
    difference() {
        cube([400, 400, base_plate_thickness], center=true);
        
        // Corner mounting holes (M10 clearance)
        for(x = [-150, 150]) {
            for(y = [-150, 150]) {
                translate([x, y, 0])
                    cylinder(h=30, d=10.5, center=true);
            }
        }
        
        // Motor mounting holes (M8 threaded, 6x on 80mm bolt circle)
        for(i = [0:60:300]) {
            rotate([0, 0, i])
                translate([40, 0, 0])
                cylinder(h=30, d=6.8, center=true);
        }
    }
}

// Simplified robot arm representation
module robot_arm_simplified() {
    // Base joint
    translate([0, 0, 0]) cylinder(h=100, d=120, center=true);
    
    // Shoulder link
    translate([0, 0, 50]) {
        rotate([0, 30, 0]) {
            translate([150, 0, 0]) cube([300, 80, 60], center=true);
            
            // Elbow joint
            translate([300, 0, 0]) {
                cylinder(h=80, d=80, center=true);
                
                // Forearm
                rotate([0, -60, 0]) {
                    translate([100, 0, 0]) cube([200, 60, 40], center=true);
                    
                    // Wrist assembly
                    translate([200, 0, 0]) {
                        cylinder(h=60, d=60, center=true);
                        translate([0, 0, 30]) cylinder(h=40, d=40, center=true);
                    }
                }
            }
        }
    }
}

// Work fixture for panel assembly
module work_fixture() {
    difference() {
        // Base plate
        cube([762, 508, 20], center=true);
        
        // Panel area recess (1mm deep)
        translate([0, 0, 9.5])
            cube([610, 305, 1], center=true);
    }
    
    // Locating pins
    translate([-305, -152.5, 10]) cylinder(h=25, d=12);
    translate([305, -152.5, 10]) cylinder(h=25, d=12);
    translate([305, 152.5, 10]) cylinder(h=25, d=12);
    translate([-305, 152.5, 10]) cylinder(h=25, d=12);
    
    // Pneumatic clamps (simplified)
    for(x = [-200, 0, 200]) {
        translate([x, 0, 20])
            cube([60, 40, 80], center=true);
    }
}

// Material feed chute
module material_feed_chute() {
    // Angled chute for gravity feed
    rotate([15, 0, 0]) {
        difference() {
            cube([100, 600, 80], center=true);
            translate([0, 0, 10])
                cube([80, 580, 60], center=true);
        }
    }
    
    // Support brackets
    for(y = [-200, 0, 200]) {
        translate([0, y, -40])
            cube([120, 40, 80], center=true);
    }
}

// Electrical enclosure
module electrical_enclosure(width, height, depth) {
    difference() {
        cube([width, height, depth]);
        translate([10, 10, 10])
            cube([width-20, height-20, depth-10]);
    }
    
    // Mounting ears
    translate([-5, height/4, 0]) cube([10, 20, depth]);
    translate([-5, 3*height/4, 0]) cube([10, 20, depth]);
}

// Pneumatic components
module pneumatic_components() {
    // Air compressor (simplified)
    cylinder(h=400, d=200);
    translate([0, 0, 400]) cylinder(h=150, d=300); // Tank
    
    // Valve manifold
    translate([250, 0, 0]) {
        cube([200, 100, 60]);
        
        // Individual valves
        for(i = [0:4]) {
            translate([20 + i*30, 0, 60])
                cube([25, 50, 40]);
        }
    }
}

// Generate the complete assembly
mini_prototype_assembly();

// Individual parts for separate printing/machining
module export_parts() {
    // Uncomment desired parts for individual export
    
    // Robot base plate
    //translate([500, 0, 0]) robot_base_assembly();
    
    // Work fixture
    //translate([0, 600, 0]) work_fixture();
    
    // T-slot extrusion sample
    //translate([0, -200, 0]) tslot_extrusion(500, 15, 15);
}

// Manufacturing annotations
module add_dimensions() {
    // Add text annotations for key dimensions
    color("red") {
        translate([frame_width/2, -50, 0])
            linear_extrude(1)
            text(str("Width: ", frame_width, "mm"), size=20, halign="center");
            
        translate([-100, frame_height/2, 0])
            rotate([0, 0, 90])
            linear_extrude(1)
            text(str("Height: ", frame_height, "mm"), size=20, halign="center");
    }
}

// Call dimension annotations
//add_dimensions();

// Main assembly function for external calls
module complete_assembly() {
    mini_prototype_assembly();
}

// Execute main assembly (comment out for 2D projections)
complete_assembly();

// Design validation
echo("Frame dimensions: ", frame_width, " x ", frame_height, " x ", frame_depth);
echo("Total volume: ", frame_width * frame_height * frame_depth / 1000000, " cubic meters");
echo("Extrusion length needed: ", 
     2 * (frame_width + frame_height) * 2 + 4 * frame_depth + frame_width + frame_height, " mm");