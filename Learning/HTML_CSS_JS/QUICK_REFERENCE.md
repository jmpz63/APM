# ROS Web HMI - Quick Reference
*Fast setup commands for APM manufacturing system*

## 🚀 **Start System (Copy & Paste)**

### **Terminal 1: Launch ROS Bridge**
```bash
# Source environment & launch rosbridge
source /opt/ros/humble/setup.bash
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091 > /tmp/rosbridge.log 2>&1 &

# Verify running
ps aux | grep rosbridge | grep -v grep
nc -z localhost 9091 && echo "✅ Ready" || echo "❌ Failed"
```

### **Terminal 2: Open HMI**  
```bash
# Navigate and launch
cd /home/arm1/APM/Learning/HTML_CSS_JS/
firefox ros_web_hmi.html &
```

## 🔧 **Troubleshooting Commands**

### **Kill & Restart**
```bash
pkill -f rosbridge; sleep 2
source /opt/ros/humble/setup.bash
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091 > /tmp/rosbridge.log 2>&1 &
```

### **Check Status**
```bash
# Processes
ps aux | grep -E "rosbridge|rosapi" | grep -v grep

# Ports  
lsof -i :9091

# Logs
tail -f /tmp/rosbridge.log
```

## 📋 **HMI Features**
- 🤖 6-DOF joint control sliders
- 🛑 Emergency stop button
- 📊 Real-time data charts  
- ⚠️ Alert notifications
- 🎮 3D robot visualization
- 📱 Mobile responsive design

## 🌐 **Connection Details**
- **WebSocket:** `ws://localhost:9091`
- **Topics:** `/joint_states`, `/joint_trajectory`  
- **Robot:** Moveo 6-DOF (configured)
- **Browser:** Firefox recommended

---
*Quick reference for APM manufacturing operations*