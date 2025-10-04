;; AutoCAD LISP Script - Mini Prototype Drawing Generator
;; Complete shop drawing automation for wall panel manufacturing system

;; Main function to create complete drawing set
(defun c:CREATE-PROTOTYPE-DRAWINGS ()
  "Create complete shop drawing set for mini prototype"
  (setvar "CMDECHO" 0)
  (command "LAYER" "M" "FRAME" "C" "CYAN" "" "")
  (command "LAYER" "M" "DIMENSIONS" "C" "RED" "" "")
  (command "LAYER" "M" "CENTERLINES" "C" "GREEN" "" "")
  (command "LAYER" "M" "HIDDEN" "L" "HIDDEN" "" "")
  (command "LAYER" "M" "TEXT" "C" "WHITE" "" "")
  
  ;; Create title block
  (create-title-block)
  
  ;; Create main assembly view
  (setvar "CLAYER" "FRAME")
  (create-overall-assembly)
  
  ;; Create detail views
  (create-robot-base-detail)
  (create-fixture-detail)
  (create-frame-detail)
  
  ;; Add dimensions
  (setvar "CLAYER" "DIMENSIONS")
  (add-overall-dimensions)
  
  (princ "\nComplete drawing set created successfully!")
  (setvar "CMDECHO" 1)
  (princ)
)

;; Title block creation
(defun create-title-block ()
  "Create standard title block in lower right corner"
  (setvar "CLAYER" "TEXT")
  
  ;; Title block border (A1 size: 594 x 841mm)
  (command "RECTANGLE" "0,0" "841,594")
  
  ;; Title block rectangle
  (command "RECTANGLE" "591,0" "841,150")
  
  ;; Internal divisions
  (command "LINE" "591,30" "841,30" "")
  (command "LINE" "591,60" "841,60" "")
  (command "LINE" "591,90" "841,90" "")
  (command "LINE" "591,120" "841,120" "")
  (command "LINE" "716,0" "716,150" "")
  (command "LINE" "778,0" "778,150" "")
  
  ;; Title text
  (command "TEXT" "596,125" "8" "0" "MINI WALL PANEL MANUFACTURING SYSTEM")
  (command "TEXT" "596,105" "6" "0" "OVERALL ASSEMBLY AND DETAILS")
  (command "TEXT" "596,85" "4" "0" "SCALE: AS NOTED")
  (command "TEXT" "596,65" "4" "0" "DATE: 2025-10-04")
  (command "TEXT" "596,45" "4" "0" "DWG NO: WP-001")
  (command "TEXT" "596,25" "4" "0" "REV: A")
  (command "TEXT" "596,5" "4" "0" "SHEET: 1 of 6")
  
  ;; Company information
  (command "TEXT" "721,125" "6" "0" "APM ENGINEERING")
  (command "TEXT" "721,105" "4" "0" "ADVANCED MANUFACTURING")
  (command "TEXT" "721,85" "4" "0" "PROTOTYPE SYSTEMS")
  
  ;; Approval blocks
  (command "TEXT" "783,125" "3" "0" "DRAWN BY:")
  (command "TEXT" "783,105" "3" "0" "CHECKED BY:")
  (command "TEXT" "783,85" "3" "0" "APPROVED BY:")
  (command "TEXT" "783,65" "3" "0" "QA:")
)

;; Overall assembly view
(defun create-overall-assembly ()
  "Create main assembly view - scale 1:50"
  (setvar "CLAYER" "FRAME")
  
  ;; Frame outline (1800 x 1200mm at 1:50 scale = 36 x 24 units)
  (setq frame-scale 0.02)  ; 1:50 scale
  (setq frame-x 100)       ; Position on drawing
  (setq frame-y 400)
  (setq frame-w (* 1800 frame-scale))  ; 36 units
  (setq frame-h (* 1200 frame-scale))  ; 24 units
  
  ;; Outer frame
  (command "RECTANGLE" 
    (strcat (rtos frame-x) "," (rtos frame-y))
    (strcat (rtos (+ frame-x frame-w)) "," (rtos (+ frame-y frame-h)))
  )
  
  ;; Internal cross members
  (command "LINE" 
    (strcat (rtos frame-x) "," (rtos (+ frame-y (/ frame-h 2))))
    (strcat (rtos (+ frame-x frame-w)) "," (rtos (+ frame-y (/ frame-h 2))))
    ""
  )
  (command "LINE"
    (strcat (rtos (+ frame-x (/ frame-w 2))) "," (rtos frame-y))
    (strcat (rtos (+ frame-x (/ frame-w 2))) "," (rtos (+ frame-y frame-h)))
    ""
  )
  
  ;; Robot base (400x400mm at center)
  (setq robot-size (* 400 frame-scale))  ; 8 units
  (setq robot-x (+ frame-x (/ (- frame-w robot-size) 2)))
  (setq robot-y (+ frame-y (/ (- frame-h robot-size) 2)))
  
  (command "RECTANGLE"
    (strcat (rtos robot-x) "," (rtos robot-y))
    (strcat (rtos (+ robot-x robot-size)) "," (rtos (+ robot-y robot-size)))
  )
  
  ;; Robot arm (simplified representation)
  (setq arm-center-x (+ robot-x (/ robot-size 2)))
  (setq arm-center-y (+ robot-y (/ robot-size 2)))
  
  (command "CIRCLE" 
    (strcat (rtos arm-center-x) "," (rtos arm-center-y))
    (rtos (* 120 frame-scale))
  )
  
  ;; Arm links (simplified)
  (command "LINE"
    (strcat (rtos arm-center-x) "," (rtos arm-center-y))
    (strcat (rtos (+ arm-center-x (* 300 frame-scale))) "," (rtos (+ arm-center-y (* 200 frame-scale))))
    ""
  )
  
  ;; Work fixture (762x508mm)
  (setq fixture-w (* 762 frame-scale))
  (setq fixture-h (* 508 frame-scale))
  (setq fixture-x (+ robot-x robot-size 50))
  (setq fixture-y (+ frame-y (/ (- frame-h fixture-h) 2)))
  
  (command "RECTANGLE"
    (strcat (rtos fixture-x) "," (rtos fixture-y))
    (strcat (rtos (+ fixture-x fixture-w)) "," (rtos (+ fixture-y fixture-h)))
  )
  
  ;; Panel outline (610x305mm)
  (setq panel-w (* 610 frame-scale))
  (setq panel-h (* 305 frame-scale))
  (setq panel-x (+ fixture-x (/ (- fixture-w panel-w) 2)))
  (setq panel-y (+ fixture-y (/ (- fixture-h panel-h) 2)))
  
  (setvar "CLAYER" "HIDDEN")
  (command "RECTANGLE"
    (strcat (rtos panel-x) "," (rtos panel-y))
    (strcat (rtos (+ panel-x panel-w)) "," (rtos (+ panel-y panel-h)))
  )
  
  ;; Add view label
  (setvar "CLAYER" "TEXT")
  (command "TEXT" 
    (strcat (rtos frame-x) "," (rtos (- frame-y 20)))
    "6" "0" "OVERALL ASSEMBLY - SCALE 1:50"
  )
)

;; Robot base detail drawing
(defun create-robot-base-detail ()
  "Create detailed view of robot base plate"
  (setvar "CLAYER" "FRAME")
  
  ;; Detail view position
  (setq detail-x 450)
  (setq detail-y 300)
  (setq detail-scale 0.25)  ; 1:4 scale for detail
  
  ;; Base plate (400x400mm)
  (setq plate-size (* 400 detail-scale))  ; 100 units
  (command "RECTANGLE"
    (strcat (rtos detail-x) "," (rtos detail-y))
    (strcat (rtos (+ detail-x plate-size)) "," (rtos (+ detail-y plate-size)))
  )
  
  ;; Corner mounting holes (M10 clearance)
  (setq hole-offset (* 150 detail-scale))
  (setq hole-dia (* 10.5 detail-scale))
  
  ;; Corner holes
  (command "CIRCLE" 
    (strcat (rtos (+ detail-x hole-offset)) "," (rtos (+ detail-y hole-offset)))
    (rtos (/ hole-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos (- (+ detail-x plate-size) hole-offset)) "," (rtos (+ detail-y hole-offset)))
    (rtos (/ hole-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos (- (+ detail-x plate-size) hole-offset)) "," (rtos (- (+ detail-y plate-size) hole-offset)))
    (rtos (/ hole-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos (+ detail-x hole-offset)) "," (rtos (- (+ detail-y plate-size) hole-offset)))
    (rtos (/ hole-dia 2))
  )
  
  ;; Motor mounting holes (6x M8 on 80mm bolt circle)
  (setq center-x (+ detail-x (/ plate-size 2)))
  (setq center-y (+ detail-y (/ plate-size 2)))
  (setq bolt-circle (* 80 detail-scale))
  (setq thread-dia (* 6.8 detail-scale))
  
  (setq angle 0)
  (repeat 6
    (setq hole-x (+ center-x (* (/ bolt-circle 2) (cos angle))))
    (setq hole-y (+ center-y (* (/ bolt-circle 2) (sin angle))))
    (command "CIRCLE"
      (strcat (rtos hole-x) "," (rtos hole-y))
      (rtos (/ thread-dia 2))
    )
    (setq angle (+ angle (/ (* 2 pi) 6)))
  )
  
  ;; Centerlines
  (setvar "CLAYER" "CENTERLINES")
  (command "LINE"
    (strcat (rtos detail-x) "," (rtos center-y))
    (strcat (rtos (+ detail-x plate-size)) "," (rtos center-y))
    ""
  )
  (command "LINE"
    (strcat (rtos center-x) "," (rtos detail-y))
    (strcat (rtos center-x) "," (rtos (+ detail-y plate-size)))
    ""
  )
  
  ;; Detail label
  (setvar "CLAYER" "TEXT")
  (command "TEXT"
    (strcat (rtos detail-x) "," (rtos (- detail-y 20)))
    "5" "0" "ROBOT BASE DETAIL - SCALE 1:4"
  )
)

;; Work fixture detail
(defun create-fixture-detail ()
  "Create work fixture detail view"
  (setvar "CLAYER" "FRAME")
  
  ;; Position for fixture detail
  (setq fixture-det-x 450)
  (setq fixture-det-y 50)
  (setq fixture-scale 0.125)  ; 1:8 scale
  
  ;; Fixture plate (762x508mm)
  (setq fixture-w (* 762 fixture-scale))
  (setq fixture-h (* 508 fixture-scale))
  
  (command "RECTANGLE"
    (strcat (rtos fixture-det-x) "," (rtos fixture-det-y))
    (strcat (rtos (+ fixture-det-x fixture-w)) "," (rtos (+ fixture-det-y fixture-h)))
  )
  
  ;; Panel area (610x305mm)
  (setq panel-w (* 610 fixture-scale))
  (setq panel-h (* 305 fixture-scale))
  (setq panel-x (+ fixture-det-x (/ (- fixture-w panel-w) 2)))
  (setq panel-y (+ fixture-det-y (/ (- fixture-h panel-h) 2)))
  
  (setvar "CLAYER" "HIDDEN")
  (command "RECTANGLE"
    (strcat (rtos panel-x) "," (rtos panel-y))
    (strcat (rtos (+ panel-x panel-w)) "," (rtos (+ panel-y panel-h)))
  )
  
  ;; Locating pins (12mm dia)
  (setvar "CLAYER" "FRAME")
  (setq pin-dia (* 12 fixture-scale))
  
  ;; Corner pins
  (command "CIRCLE"
    (strcat (rtos panel-x) "," (rtos panel-y))
    (rtos (/ pin-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos (+ panel-x panel-w)) "," (rtos panel-y))
    (rtos (/ pin-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos (+ panel-x panel-w)) "," (rtos (+ panel-y panel-h)))
    (rtos (/ pin-dia 2))
  )
  (command "CIRCLE"
    (strcat (rtos panel-x) "," (rtos (+ panel-y panel-h)))
    (rtos (/ pin-dia 2))
  )
  
  ;; Clamps (simplified representation)
  (setq clamp-w (* 60 fixture-scale))
  (setq clamp-h (* 40 fixture-scale))
  
  ;; Center clamp
  (command "RECTANGLE"
    (strcat (rtos (- (+ fixture-det-x (/ fixture-w 2)) (/ clamp-w 2))) "," (rtos (+ fixture-det-y fixture-h 10)))
    (strcat (rtos (+ (+ fixture-det-x (/ fixture-w 2)) (/ clamp-w 2))) "," (rtos (+ fixture-det-y fixture-h 10 clamp-h)))
  )
  
  ;; Detail label
  (setvar "CLAYER" "TEXT")
  (command "TEXT"
    (strcat (rtos fixture-det-x) "," (rtos (- fixture-det-y 20)))
    "4" "0" "WORK FIXTURE DETAIL - SCALE 1:8"
  )
)

;; Frame construction detail
(defun create-frame-detail ()
  "Create T-slot frame connection detail"
  (setvar "CLAYER" "FRAME")
  
  ;; Detail position
  (setq frame-det-x 650)
  (setq frame-det-y 350)
  (setq frame-det-scale 2.0)  ; 2:1 scale for detail
  
  ;; T-slot extrusion (15x15mm)
  (setq extrusion-size (* 15 frame-det-scale))
  
  ;; Horizontal extrusion
  (command "RECTANGLE"
    (strcat (rtos frame-det-x) "," (rtos frame-det-y))
    (strcat (rtos (+ frame-det-x (* 50 frame-det-scale))) "," (rtos (+ frame-det-y extrusion-size)))
  )
  
  ;; Vertical extrusion
  (command "RECTANGLE"
    (strcat (rtos frame-det-x) "," (rtos frame-det-y))
    (strcat (rtos (+ frame-det-x extrusion-size)) "," (rtos (+ frame-det-y (* 50 frame-det-scale))))
  )
  
  ;; T-slot grooves
  (setvar "CLAYER" "HIDDEN")
  (setq groove-w (* 6 frame-det-scale))
  (setq groove-d (* 3 frame-det-scale))
  
  ;; Top groove
  (command "RECTANGLE"
    (strcat (rtos (+ frame-det-x (/ (- extrusion-size groove-w) 2))) "," (rtos (- (+ frame-det-y extrusion-size) groove-d)))
    (strcat (rtos (+ frame-det-x (/ (+ extrusion-size groove-w) 2))) "," (rtos (+ frame-det-y extrusion-size)))
  )
  
  ;; Corner bracket
  (setvar "CLAYER" "FRAME")
  (setq bracket-size (* 20 frame-det-scale))
  
  (command "RECTANGLE"
    (strcat (rtos (+ frame-det-x extrusion-size)) "," (rtos (+ frame-det-y extrusion-size)))
    (strcat (rtos (+ frame-det-x extrusion-size bracket-size)) "," (rtos (+ frame-det-y extrusion-size bracket-size)))
  )
  
  ;; Fastener (M6 socket head cap screw)
  (setq screw-dia (* 6 frame-det-scale))
  (command "CIRCLE"
    (strcat (rtos (+ frame-det-x extrusion-size (/ bracket-size 2))) "," (rtos (+ frame-det-y (/ extrusion-size 2))))
    (rtos (/ screw-dia 2))
  )
  
  ;; Detail label
  (setvar "CLAYER" "TEXT")
  (command "TEXT"
    (strcat (rtos frame-det-x) "," (rtos (- frame-det-y 20)))
    "4" "0" "FRAME CONNECTION DETAIL - SCALE 2:1"
  )
)

;; Add overall dimensions
(defun add-overall-dimensions ()
  "Add key dimensions to assembly view"
  (setvar "CLAYER" "DIMENSIONS")
  (setvar "DIMSCALE" 1.0)
  (setvar "DIMTXT" 4)
  
  ;; Frame overall dimensions
  (setq frame-x 100)
  (setq frame-y 400)
  (setq frame-w 36)
  (setq frame-h 24)
  
  ;; Width dimension
  (command "DIMLINEAR"
    (strcat (rtos frame-x) "," (rtos frame-y))
    (strcat (rtos (+ frame-x frame-w)) "," (rtos frame-y))
    (strcat (rtos (+ frame-x (/ frame-w 2))) "," (rtos (- frame-y 30)))
    "1800"
  )
  
  ;; Height dimension
  (command "DIMLINEAR"
    (strcat (rtos frame-x) "," (rtos frame-y))
    (strcat (rtos frame-x) "," (rtos (+ frame-y frame-h)))
    (strcat (rtos (- frame-x 30)) "," (rtos (+ frame-y (/ frame-h 2))))
    "1200"
  )
  
  ;; Robot base dimensions (detail view)
  (setq detail-x 450)
  (setq detail-y 300)
  (setq plate-size 100)
  
  (command "DIMLINEAR"
    (strcat (rtos detail-x) "," (rtos detail-y))
    (strcat (rtos (+ detail-x plate-size)) "," (rtos detail-y))
    (strcat (rtos (+ detail-x (/ plate-size 2))) "," (rtos (- detail-y 40)))
    "400"
  )
  
  (command "DIMLINEAR"
    (strcat (rtos detail-x) "," (rtos detail-y))
    (strcat (rtos detail-x) "," (rtos (+ detail-y plate-size)))
    (strcat (rtos (- detail-x 40)) "," (rtos (+ detail-y (/ plate-size 2))))
    "400"
  )
)

;; Utility functions
(defun create-bolt-circle (center-x center-y radius count dia /)
  "Create bolt circle with specified parameters"
  (setq angle-step (/ (* 2 pi) count))
  (setq current-angle 0)
  (repeat count
    (setq hole-x (+ center-x (* radius (cos current-angle))))
    (setq hole-y (+ center-y (* radius (sin current-angle))))
    (command "CIRCLE"
      (strcat (rtos hole-x) "," (rtos hole-y))
      (rtos (/ dia 2))
    )
    (setq current-angle (+ current-angle angle-step))
  )
)

(defun add-hatch-pattern (boundary-objects pattern /)
  "Add hatch pattern to closed boundary"
  (command "HATCH" pattern "" "S" boundary-objects "" "")
)

;; Load message
(princ "\nMini Prototype Drawing Commands Loaded:")
(princ "\n  CREATE-PROTOTYPE-DRAWINGS - Generate complete drawing set")
(princ "\n  Type command name to execute")
(princ)