#!/usr/bin/env python3
"""
APM AI Agent Interface - Simple, Clean API
Provides easy access to the unified APM workflow system for AI agents

This replaces the old ai_agent_helper.py with a cleaner, simpler interface.
"""

from pathlib import Path
import sys

# Portable root resolution
try:
    from config.apm_config import get_apm_root
    APM_ROOT = get_apm_root()
except Exception:
    APM_ROOT = Path(__file__).resolve().parent.parent

# Ensure _ADMIN is importable without hardcoded absolute path
_admin_path = APM_ROOT / "_ADMIN"
if str(_admin_path) not in sys.path:
    sys.path.insert(0, str(_admin_path))

from apm_unified_system import APMWorkflowSystem

class AIAgentInterface:
    """Simple interface for AI agents to interact with APM system"""
    
    def __init__(self):
        self.system = APMWorkflowSystem()
    
    def notify_knowledge(self, file_path, description="", priority="medium"):
        """
        Notify system of new knowledge creation/modification
        
        Args:
            file_path (str): Path to the knowledge file
            description (str): Brief description of the content
            priority (str): Priority level - 'high', 'medium', or 'low'
        
        Returns:
            bool: True if successful, False if failed
        """
        print(f"📝 AI Agent: Registering knowledge - {Path(file_path).name}")
        return self.system.process_knowledge_operation(file_path, description, priority)
    
    def ensure_compliance(self):
        """
        Ensure full system compliance - run complete workflow
        
        Returns:
            bool: True if system is compliant, False if issues detected
        """
        print("🤖 AI Agent: Ensuring system compliance...")
        return self.system.run_full_workflow("auto")
    
    def check_status(self):
        """
        Get current system status
        
        Returns:
            dict: System status information
        """
        return self.system.get_system_status()
    
    def print_status_report(self):
        """Print a formatted status report"""
        status = self.check_status()
        
        print("🤖 AI Agent System Status")
        print("=" * 30)
        print(f"Health: {'✅ ' + status['system_health'] if status['system_health'] == 'healthy' else '⚠️ ' + status['system_health']}")
        print(f"Tracked Files: {status['total_files']}")
        print(f"Total Operations: {status['total_operations']}")
        print(f"Integration Status: {status['integration_status']}")
        
        if status['last_push'] > 0:
            from datetime import datetime
            last_push = datetime.fromtimestamp(status['last_push'])
            print(f"Last Push: {last_push.strftime('%Y-%m-%d %H:%M')}")
        else:
            print("Last Push: Never")

# === SIMPLE CONVENIENCE FUNCTIONS FOR AI AGENTS ===

def notify_new_document(file_path, description="", priority="medium"):
    """
    Simple function for AI agents to notify of new documents
    
    Usage:
        notify_new_document("Knowledge_Base/new_analysis.md", "STM32 analysis", "high")
    """
    interface = AIAgentInterface()
    return interface.notify_knowledge(file_path, description, priority)

def ensure_compliance():
    """
    Simple function to ensure full system compliance
    
    Usage:
        ensure_compliance()  # Run after any knowledge operations
    """
    interface = AIAgentInterface()
    return interface.ensure_compliance()

def quick_status():
    """
    Quick status check and report
    
    Usage:
        quick_status()  # Print current system status
    """
    interface = AIAgentInterface()
    interface.print_status_report()

def auto_integrate():
    """
    Automatically integrate any pending changes
    
    Usage:
        auto_integrate()  # Process all pending knowledge operations
    """
    interface = AIAgentInterface()
    status = interface.check_status()
    
    if status['integration_status'] in ['idle', 'pending']:
        print("🤖 AI Agent: Running auto-integration...")
        return interface.ensure_compliance()
    else:
        print("✅ AI Agent: System already up to date")
        return True

# === CLI INTERFACE ===

def main():
    """CLI interface for direct script execution"""
    import sys
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "status":
            quick_status()
        elif command == "integrate":
            auto_integrate()
        elif command == "compliance":
            ensure_compliance()
        elif command == "help":
            print("""
APM AI Agent Interface Commands:

python3 ai_agent_interface.py status      - Show system status
python3 ai_agent_interface.py integrate   - Auto-integrate pending changes  
python3 ai_agent_interface.py compliance  - Ensure full compliance
python3 ai_agent_interface.py help        - Show this help

Python API:
    from ai_agent_interface import notify_new_document, ensure_compliance
    
    # After creating knowledge:
    notify_new_document("path/to/file.md", "description", "priority")
    ensure_compliance()
""")
        else:
            print(f"Unknown command: {command}")
            print("Use 'help' for available commands")
    else:
        # Default: show status
        quick_status()

if __name__ == "__main__":
    main()