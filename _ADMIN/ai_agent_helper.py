#!/usr/bin/env python3
"""
AI Agent Integration Helper
Ensures AI agents always follow the proper workflow when acquiring new knowledge

This script should be called by AI agents whenever they:
1. Create new documentation
2. Add knowledge files  
3. Update existing content
4. Complete analysis or research

Usage:
    from ai_agent_helper import AIAgentHelper
    
    helper = AIAgentHelper()
    helper.notify_knowledge_acquisition("path/to/new/file.md", "robotics analysis")
    helper.ensure_workflow_compliance()
"""

import os
import sys
import json
import subprocess
from pathlib import Path
from datetime import datetime, timezone

class AIAgentHelper:
    def __init__(self):
        self.apm_root = Path("/home/arm1/APM")
        self.workflow_script = self.apm_root / "_ADMIN" / "ai_knowledge_workflow.py" 
        self.agent_log = self.apm_root / "_ADMIN" / "ai_agent_activity.json"
        
        # Load or create agent activity log
        self.activity_log = self.load_activity_log()
        
    def load_activity_log(self):
        """Load the AI agent activity log"""
        if self.agent_log.exists():
            with open(self.agent_log, 'r') as f:
                return json.load(f)
        else:
            return {
                "version": "1.0",
                "activities": [],
                "last_workflow_run": 0,
                "compliance_status": "unknown"
            }
    
    def save_activity_log(self):
        """Save the AI agent activity log"""
        with open(self.agent_log, 'w') as f:
            json.dump(self.activity_log, f, indent=2)
    
    def notify_knowledge_acquisition(self, file_path, description="", priority="medium"):
        """Notify system of new knowledge acquisition"""
        activity = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "action": "knowledge_acquisition", 
            "file_path": str(file_path),
            "description": description,
            "priority": priority,
            "workflow_triggered": False,
            "integration_status": "pending"
        }
        
        self.activity_log["activities"].append(activity)
        self.save_activity_log()
        
        print(f"📝 AI Agent: Registered knowledge acquisition - {Path(file_path).name}")
        
        # Auto-trigger workflow if high priority
        if priority == "high":
            self.trigger_workflow()
    
    def notify_content_update(self, file_path, change_type="modification"):
        """Notify system of content updates"""
        activity = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "action": "content_update",
            "file_path": str(file_path), 
            "change_type": change_type,
            "workflow_triggered": False,
            "integration_status": "pending"
        }
        
        self.activity_log["activities"].append(activity)
        self.save_activity_log()
        
        print(f"📝 AI Agent: Registered content update - {Path(file_path).name}")
    
    def ensure_workflow_compliance(self, force=False):
        """Ensure AI agent is following proper workflow"""
        print("🤖 AI Agent: Ensuring workflow compliance...")
        
        # Check for pending activities
        pending_activities = [a for a in self.activity_log["activities"] 
                            if a.get("integration_status") == "pending"]
        
        if not pending_activities and not force:
            print("✅ AI Agent: No pending activities - compliance maintained")
            return True
        
        print(f"📋 AI Agent: {len(pending_activities)} pending activities detected")
        
        # Trigger workflow
        return self.trigger_workflow()
    
    def trigger_workflow(self):
        """Trigger the AI knowledge workflow"""
        print("🚀 AI Agent: Triggering knowledge workflow...")
        
        try:
            # Change to APM directory
            os.chdir(self.apm_root)
            
            # Run workflow in auto mode
            result = subprocess.run([
                sys.executable, str(self.workflow_script), "--mode", "auto"
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode == 0:
                print("✅ AI Agent: Workflow completed successfully")
                
                # Mark all pending activities as integrated
                for activity in self.activity_log["activities"]:
                    if activity.get("integration_status") == "pending":
                        activity["integration_status"] = "integrated"
                        activity["workflow_triggered"] = True
                
                self.activity_log["last_workflow_run"] = datetime.now(timezone.utc).timestamp()
                self.activity_log["compliance_status"] = "compliant"
                self.save_activity_log()
                
                return True
            else:
                print(f"❌ AI Agent: Workflow failed - {result.stderr}")
                self.activity_log["compliance_status"] = "workflow_failed"
                self.save_activity_log()
                return False
                
        except subprocess.TimeoutExpired:
            print("⏰ AI Agent: Workflow timeout - manual intervention may be needed")
            return False
        except Exception as e:
            print(f"❌ AI Agent: Workflow error - {e}")
            return False
    
    def get_compliance_status(self):
        """Get current compliance status"""
        pending = len([a for a in self.activity_log["activities"] 
                      if a.get("integration_status") == "pending"])
        
        status = {
            "compliant": pending == 0,
            "pending_activities": pending,
            "last_workflow_run": self.activity_log.get("last_workflow_run", 0),
            "status": self.activity_log.get("compliance_status", "unknown")
        }
        
        return status
    
    def print_compliance_report(self):
        """Print a compliance status report"""
        status = self.get_compliance_status()
        
        print("🤖 AI Agent Compliance Report")
        print("=" * 30)
        print(f"Status: {'✅ Compliant' if status['compliant'] else '⚠️ Non-Compliant'}")
        print(f"Pending Activities: {status['pending_activities']}")
        
        if status['last_workflow_run'] > 0:
            last_run = datetime.fromtimestamp(status['last_workflow_run'])
            print(f"Last Workflow Run: {last_run}")
        else:
            print("Last Workflow Run: Never")
        
        print(f"Integration Status: {status['status']}")
        
        # Show recent activities
        recent_activities = self.activity_log["activities"][-5:]
        if recent_activities:
            print("\nRecent Activities:")
            for activity in recent_activities:
                timestamp = datetime.fromisoformat(activity["timestamp"]).strftime("%Y-%m-%d %H:%M")
                status_icon = "✅" if activity.get("integration_status") == "integrated" else "⏳"
                print(f"  {status_icon} {timestamp} - {activity['action']} - {Path(activity['file_path']).name}")

# Convenience functions for AI agents
def notify_new_document(file_path, description="", priority="medium"):
    """Quick function for AI agents to notify of new documents"""
    helper = AIAgentHelper()
    helper.notify_knowledge_acquisition(file_path, description, priority)
    
def ensure_compliance():
    """Quick function for AI agents to ensure compliance"""
    helper = AIAgentHelper()
    return helper.ensure_workflow_compliance()

def auto_integrate_if_needed():
    """Automatically integrate if there are pending activities"""
    helper = AIAgentHelper()
    status = helper.get_compliance_status()
    
    if not status["compliant"]:
        print(f"🤖 AI Agent: Auto-integrating {status['pending_activities']} pending activities...")
        return helper.trigger_workflow()
    else:
        print("✅ AI Agent: Already compliant - no integration needed")
        return True

def quick_status():
    """Quick status check"""
    helper = AIAgentHelper()
    helper.print_compliance_report()

# Auto-run compliance check if script is executed directly
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "status":
            quick_status()
        elif sys.argv[1] == "integrate":
            auto_integrate_if_needed()
        elif sys.argv[1] == "ensure":
            ensure_compliance()
        else:
            print("Usage: python3 ai_agent_helper.py [status|integrate|ensure]")
    else:
        # Default behavior - ensure compliance
        helper = AIAgentHelper()
        helper.print_compliance_report()
        
        status = helper.get_compliance_status()
        if not status["compliant"]:
            response = input(f"🤖 Trigger workflow for {status['pending_activities']} pending activities? (y/N): ")
            if response.lower() == 'y':
                helper.trigger_workflow()