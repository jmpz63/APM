# Quick Reference Card - Shop Drawing System
*Essential commands and information for the next developer*

## 🚀 **QUICK START (5 minutes)**

### **Generate Drawings Right Now**
```bash
# Single command for complete drawing package:
/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/fixed_shop_drawings.sh

# View results:
firefox /home/arm1/APM/Projects/Active/Mini_Prototype/Drawings/technical_drawings_enhanced.html
```

### **Key File Locations**
```
📁 /home/arm1/APM/Projects/Active/Mini_Prototype/
├── 📄 CAD/mini_prototype_model.scad          # Source 3D model
├── 🔧 CAD/fixed_shop_drawings.sh             # MAIN GENERATOR (use this)
├── 📊 Drawings/technical_drawings_enhanced.pdf # Output PDF
└── 📋 CAD/LESSONS_LEARNED.md                 # READ THIS FIRST
```

---

## ⚡ **CRITICAL SUCCESS COMMANDS**

### **Working OpenSCAD Render**
```bash
# ✅ RELIABLE: High-quality orthographic view
openscad --camera=900,2200,600,60,0,0,2500 --imgsize=1600,1200 \
         --projection=orthogonal --render -o "output.png" "model.scad"
```

### **Working PDF Generation**  
```bash
# ✅ RELIABLE: HTML to professional PDF
wkhtmltopdf --page-size A3 --orientation Portrait \
            --enable-local-file-access --load-error-handling ignore \
            "source.html" "output.pdf"
```

### **Model Structure Required**
```openscad
// ✅ REQUIRED: Proper OpenSCAD model structure
module complete_assembly() {
    mini_prototype_assembly();  // Your main model here
}
complete_assembly();  // Execute for rendering
```

---

## ❌ **AVOID THESE (Time Wasters)**

```bash
# ❌ DON'T USE: FreeCAD integration (broken dependencies)
freecad --run-python script.py

# ❌ DON'T USE: Complex 2D projections (unreliable)  
projection(cut=false) { complex_model(); }

# ❌ DON'T USE: Colorscheme parameters (causes crashes)
openscad --colorscheme=WhiteBackground
```

---

## 🔧 **DEBUG CHECKLIST**

1. **Model won't render?**
   - Check OpenSCAD GUI first: `openscad model.scad`
   - Look for syntax errors in console

2. **Images blank/small?**
   - Verify camera position: `--camera=X,Y,Z,rotX,rotY,rotZ,distance`
   - Check model bounds: Echo dimensions in SCAD

3. **PDF missing images?**
   - Verify file paths are absolute
   - Check `--enable-local-file-access` flag

4. **Quality issues?**
   - Target PNG size: 10-20KB (good quality)
   - Use 1600x1200 resolution minimum

---

## 📊 **QUALITY METRICS**

### **Good Results**
- ✅ PDF: 200-500KB (professional layout)
- ✅ PNGs: 10-20KB each (clear geometry)  
- ✅ Render time: <30 seconds per view
- ✅ HTML loads: <5 seconds

### **Problem Indicators**
- ❌ PNG <5KB: Blank render (fix camera)
- ❌ PDF <50KB: Missing content (check HTML)
- ❌ Render >60sec: Model too complex (simplify)

---

## 🎯 **NEXT STEPS (Priority Order)**

1. **Fix image quality** - Improve front/iso view cameras
2. **Add progress bars** - User feedback during rendering  
3. **Automatic dimensions** - Parse model for key measurements
4. **Section views** - Internal component visibility

---

## 📚 **ESSENTIAL READING**

1. **LESSONS_LEARNED.md** - What works vs what doesn't ⭐
2. **NEXT_STEPS_ROADMAP.md** - Strategic development path
3. **FINAL_SYSTEM_DOCUMENTATION.md** - Complete system overview

---

## 🆘 **EMERGENCY CONTACTS**

```bash
# System broken? Reset to working state:
git checkout HEAD -- CAD/fixed_shop_drawings.sh

# Start fresh:
cd /home/arm1/APM/Projects/Active/Mini_Prototype/CAD
./fixed_shop_drawings.sh

# Check git history for working versions:
git log --oneline | grep -i "shop\|drawing"
```

---

## 🏆 **SUCCESS DEFINITION**

**You're winning when:**
- Single command generates professional PDF
- Images clearly show model geometry  
- Manufacturing team can use drawings as-is
- System works reliably without manual intervention

**Built on solid foundation - enhance confidently!** 🚀