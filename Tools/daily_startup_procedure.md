# Daily Startup Procedure - Beelink & Robot System

## Quick Start Checklist

### 1. Beelink System Power-On
- [ ] Power on Beelink mini PC
- [ ] Wait for system boot (30-60 seconds)
- [ ] Verify network connectivity
- [ ] Check system status indicators

### 2. Network Connectivity Check
```bash
# Test robot connectivity
ping 192.168.50.11

# Expected response:
# PING 192.168.50.11 (192.168.50.11) 56(84) bytes of data.
# 64 bytes from 192.168.50.11: icmp_seq=1 ttl=64 time=0.234 ms
```

### 3. SSH Connection to Robot
```bash
# Quick connection
ssh arm1@192.168.50.11

# Or using connection script
~/connect_robot.sh
```

### 4. APM System Verification
```bash
# On robot - check APM status
cd /home/arm1/APM
python3 _ADMIN/ai_agent_interface.py status

# Expected output:
# 🤖 AI Agent Compliance Report
# Status: ✅ Compliant
# Pending Activities: 0
```

### 5. VS Code Development Environment
```bash
# Start VS Code with remote connection
code --remote ssh-remote+robot-arm1 /home/arm1/APM

# Or open locally then connect
code ~/Documents/APM
# Then: Ctrl+Shift+P → "Remote-SSH: Connect to Host" → "robot-arm1"
```

### 6. Optional: Sync APM Knowledge Base
```bash
# Update local APM copy
cd ~/Documents/APM
git pull origin master

# Update robot APM copy (if needed)
ssh arm1@192.168.50.11 "cd /home/arm1/APM && git pull origin master"
```

## Automated Startup (Windows)

### PowerShell Startup Script
Save as `robot_startup.ps1`:
```powershell
# Robot Development Environment - Daily Startup
Write-Host "🤖 Robot Development Environment Startup" -ForegroundColor Green
Write-Host "=======================================" -ForegroundColor Cyan

# 1. Test robot connectivity
Write-Host "Testing robot connectivity..." -ForegroundColor Yellow
if (Test-Connection -ComputerName 192.168.50.11 -Count 1 -Quiet) {
    Write-Host "✅ Robot ARM1 is online (192.168.50.11)" -ForegroundColor Green
    
    # 2. Test SSH connection
    Write-Host "Testing SSH connection..." -ForegroundColor Yellow
    $sshTest = ssh arm1@192.168.50.11 "echo 'SSH connection successful'"
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ SSH connection verified" -ForegroundColor Green
        
        # 3. Check APM system
        Write-Host "Checking APM system status..." -ForegroundColor Yellow
    ssh arm1@192.168.50.11 "cd /home/arm1/APM && python3 _ADMIN/ai_agent_interface.py status"
        
    } else {
        Write-Host "❌ SSH connection failed" -ForegroundColor Red
    }
    
} else {
    Write-Host "❌ Robot ARM1 is offline" -ForegroundColor Red
    Write-Host "   Check robot power and network connection" -ForegroundColor Yellow
    exit 1
}

# 4. Update local APM
Write-Host "Updating local APM knowledge base..." -ForegroundColor Yellow
Set-Location "$HOME\Documents\APM"
git pull origin master

# 5. Start VS Code
Write-Host "Starting VS Code development environment..." -ForegroundColor Yellow
Start-Process "code" -ArgumentList "$HOME\Documents\APM"

Write-Host "✅ Startup complete! Development environment ready." -ForegroundColor Green
```

### Linux/Ubuntu Startup Script
Save as `robot_startup.sh`:
```bash
#!/bin/bash
# Robot Development Environment - Daily Startup

echo "🤖 Robot Development Environment Startup"
echo "======================================="

# 1. Test robot connectivity
echo "Testing robot connectivity..."
if ping -c 1 192.168.50.11 >/dev/null 2>&1; then
    echo "✅ Robot ARM1 is online (192.168.50.11)"
    
    # 2. Test SSH connection
    echo "Testing SSH connection..."
    if ssh -o ConnectTimeout=5 arm1@192.168.50.11 "echo 'SSH connection successful'" >/dev/null 2>&1; then
        echo "✅ SSH connection verified"
        
        # 3. Check APM system
        echo "Checking APM system status..."
    ssh arm1@192.168.50.11 "cd /home/arm1/APM && python3 _ADMIN/ai_agent_interface.py status"
        
    else
        echo "❌ SSH connection failed"
    fi
    
else
    echo "❌ Robot ARM1 is offline"
    echo "   Check robot power and network connection"
    exit 1
fi

# 4. Update local APM
echo "Updating local APM knowledge base..."
cd ~/Documents/APM
git pull origin master

# 5. Start VS Code (if GUI available)
if [ -n "$DISPLAY" ]; then
    echo "Starting VS Code development environment..."
    code ~/Documents/APM &
fi

echo "✅ Startup complete! Development environment ready."
```

## Troubleshooting Quick Fixes

### Robot Not Reachable
```bash
# Check Beelink network configuration
ip addr show    # Linux
ipconfig        # Windows

# Restart network interface
sudo systemctl restart NetworkManager  # Linux
ipconfig /renew                        # Windows

# Check router/switch connection
ping 192.168.50.1  # Gateway
```

### SSH Connection Issues
```bash
# Regenerate SSH keys if needed
ssh-keygen -t rsa -b 4096 -C "beelink@robot-network"
ssh-copy-id arm1@192.168.50.11

# Clear known hosts if key changed
ssh-keygen -R 192.168.50.11
```

### APM System Not Responding
```bash
# Check APM directory exists
ssh arm1@192.168.50.11 "ls -la /home/arm1/APM"

# Reset APM workflow if needed
ssh arm1@192.168.50.11 "cd /home/arm1/APM && rm -f _ADMIN/knowledge_workflow_state.json"

# Test APM manually
ssh arm1@192.168.50.11 "cd /home/arm1/APM && python3 _ADMIN/ai_knowledge_workflow.py --mode manual"
```

### VS Code Remote Connection Problems
```bash
# Clear VS Code remote cache
rm -rf ~/.vscode-server  # Linux
Remove-Item -Recurse "$env:USERPROFILE\.vscode-server"  # Windows

# Restart VS Code and reconnect
```

## Status Indicators

### Green Light (All Systems Go)
- ✅ Robot responds to ping
- ✅ SSH connection successful
- ✅ APM system compliant
- ✅ VS Code connects remotely

### Yellow Light (Caution)
- ⚠️ Robot reachable but APM has issues
- ⚠️ SSH works but slow response
- ⚠️ Local APM out of sync

### Red Light (Action Required)
- ❌ Robot not reachable
- ❌ SSH connection fails
- ❌ APM system errors
- ❌ Network configuration issues

---

**Created**: $(date '+%Y-%m-%d %H:%M:%S')
**Purpose**: Daily operations and troubleshooting
**Use**: Run startup script each development session
**Updated**: $(date)
