#!/bin/bash
"""
AI Knowledge Workflow Quick Start Script
Provides easy access to the enhanced AI knowledge workflow system
"""

APM_ROOT="/home/arm1/APM"
WORKFLOW_SCRIPT="$APM_ROOT/_ADMIN/ai_knowledge_workflow.py"

echo "🤖 APM AI Knowledge Workflow System"
echo "=================================="

# Check if workflow script exists
if [ ! -f "$WORKFLOW_SCRIPT" ]; then
    echo "❌ Workflow script not found: $WORKFLOW_SCRIPT"
    exit 1
fi

# Make script executable
chmod +x "$WORKFLOW_SCRIPT"

# Display menu
echo ""
echo "Select workflow mode:"
echo "1) 🚀 AUTO    - Fully automated detection, analysis, indexing, and push"
echo "2) 🔧 MANUAL  - Step-by-step with user confirmation"
echo "3) 👁️  MONITOR - Detect changes only (no actions)"
echo "4) 📊 STATUS  - Show current workflow status"
echo "5) ⚙️  CONFIG  - Edit workflow configuration"
echo ""

read -p "Enter choice (1-5): " choice

case $choice in
    1)
        echo "🚀 Running AUTO workflow..."
        cd "$APM_ROOT"
        python3 "$WORKFLOW_SCRIPT" --mode auto
        ;;
    2)
        echo "🔧 Running MANUAL workflow..."
        cd "$APM_ROOT"
        python3 "$WORKFLOW_SCRIPT" --mode manual
        ;;
    3)
        echo "👁️ Running MONITOR mode..."
        cd "$APM_ROOT"
        python3 "$WORKFLOW_SCRIPT" --mode monitor
        ;;
    4)
        echo "📊 Workflow Status:"
        cd "$APM_ROOT"
        
        # Check git status
        echo ""
        echo "Git Status:"
        git status --short
        
        # Check workflow state file
        if [ -f "$APM_ROOT/_ADMIN/knowledge_workflow_state.json" ]; then
            echo ""
            echo "Last Workflow Run:"
            python3 -c "
import json
from datetime import datetime
try:
    with open('$APM_ROOT/_ADMIN/knowledge_workflow_state.json', 'r') as f:
        state = json.load(f)
    
    last_scan = datetime.fromtimestamp(state.get('last_scan_timestamp', 0))
    last_push = datetime.fromtimestamp(state.get('last_push_timestamp', 0))
    
    print(f'  Last Scan: {last_scan}')
    print(f'  Last Push: {last_push}')
    print(f'  Status: {state.get(\"integration_status\", \"unknown\")}')
    print(f'  Tracked Files: {len(state.get(\"file_checksums\", {}))}')
except Exception as e:
    print(f'  Status file not available: {e}')
"
        else
            echo "  No workflow state available"
        fi
        ;;
    5)
        echo "⚙️ Opening workflow configuration..."
        if command -v code &> /dev/null; then
            code "$APM_ROOT/_ADMIN/ai_workflow_config.json"
        else
            nano "$APM_ROOT/_ADMIN/ai_workflow_config.json"
        fi
        ;;
    *)
        echo "❌ Invalid choice. Exiting."
        exit 1
        ;;
esac

echo ""
echo "✅ Workflow operation completed"
echo "💡 Run 'bash $APM_ROOT/_ADMIN/workflow.sh' anytime to access this menu"