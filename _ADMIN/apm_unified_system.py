#!/usr/bin/env python3
"""
APM Unified Workflow System
Consolidated automation engine combining all workflow, maintenance, and knowledge operations

This replaces and consolidates:
- ai_knowledge_workflow.py
- ai_assistant_automation.py  
- knowledge_expansion_automation.py

Single, comprehensive system for all APM operations.
"""

import os
import json
import subprocess
import hashlib
import time
from pathlib import Path
from datetime import datetime, timezone
import argparse
import shutil

class APMWorkflowSystem:
    """Unified APM automation and workflow system"""
    
    def __init__(self):
        # Portable root resolution (no hardcoded Linux path)
        try:
            from config.apm_config import get_apm_root
            self.apm_root = get_apm_root()
        except Exception:
            # Fallback: assume parent of _ADMIN directory is root
            self.apm_root = Path(__file__).resolve().parent.parent
        self.knowledge_base = self.apm_root / "Knowledge_Base"
        self.admin_dir = self.apm_root / "_ADMIN"
        
        # State files
        self.workflow_state_file = self.admin_dir / "apm_system_state.json"
        self.activity_log_file = self.admin_dir / "apm_activity_log.json"
        
        # Index files
        self.master_index = self.apm_root / "00_MASTER_KNOWLEDGE_INDEX.md"
        
        # Load system state
        self.system_state = self.load_system_state()
        self.activity_log = self.load_activity_log()
        
    def load_system_state(self):
        """Load unified system state"""
        if self.workflow_state_file.exists():
            with open(self.workflow_state_file, 'r') as f:
                return json.load(f)
        else:
            return {
                "version": "2.0_unified",
                "last_scan_timestamp": 0,
                "file_checksums": {},
                "knowledge_items": [],
                "integration_status": "idle",
                "last_push_timestamp": 0,
                "last_maintenance": 0,
                "maintenance_tasks": [],
                "system_health": "unknown"
            }
    
    def save_system_state(self):
        """Save unified system state"""
        self.system_state["last_scan_timestamp"] = datetime.now(timezone.utc).timestamp()
        with open(self.workflow_state_file, 'w') as f:
            json.dump(self.system_state, f, indent=2)
    
    def load_activity_log(self):
        """Load activity log"""
        if self.activity_log_file.exists():
            with open(self.activity_log_file, 'r') as f:
                return json.load(f)
        else:
            return {
                "version": "2.0_unified",
                "activities": [],
                "total_operations": 0,
                "compliance_status": "unknown"
            }
    
    def save_activity_log(self):
        """Save activity log"""
        with open(self.activity_log_file, 'w') as f:
            json.dump(self.activity_log, f, indent=2)
    
    # === KNOWLEDGE WORKFLOW OPERATIONS ===
    
    def detect_knowledge_changes(self):
        """Detect new/modified knowledge files"""
        print("🔍 Scanning for knowledge changes...")
        
        knowledge_dirs = [
            self.knowledge_base,
            self.apm_root / "Projects",
            self.apm_root / "Research", 
            self.apm_root / "Learning",
            self.apm_root / "Engineering"
        ]
        
        current_files = {}
        new_files = []
        modified_files = []
        
        for directory in knowledge_dirs:
            if directory.exists():
                for file_path in directory.rglob("*.md"):
                    if file_path.is_file():
                        with open(file_path, 'rb') as f:
                            checksum = hashlib.md5(f.read()).hexdigest()
                        
                        relative_path = str(file_path.relative_to(self.apm_root))
                        current_files[relative_path] = {
                            "checksum": checksum,
                            "timestamp": file_path.stat().st_mtime,
                            "size": file_path.stat().st_size
                        }
                        
                        if relative_path not in self.system_state["file_checksums"]:
                            new_files.append(relative_path)
                        elif self.system_state["file_checksums"][relative_path]["checksum"] != checksum:
                            modified_files.append(relative_path)
        
        self.system_state["file_checksums"] = current_files
        
        if new_files:
            print(f"📄 New files: {len(new_files)}")
        if modified_files:
            print(f"📝 Modified files: {len(modified_files)}")
        
        return new_files, modified_files
    
    def analyze_content(self, file_paths):
        """Analyze content for categorization and indexing"""
        print("🧠 Analyzing content...")
        
        analyses = []
        for file_path in file_paths:
            try:
                full_path = self.apm_root / file_path
                with open(full_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                analysis = {
                    "file_path": file_path,
                    "word_count": len(content.split()),
                    "line_count": len(content.splitlines()),
                    "categories": self.categorize_content(content),
                    "priority": self.assess_priority(content, file_path),
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                analyses.append(analysis)
                
            except Exception as e:
                print(f"⚠️ Error analyzing {file_path}: {e}")
        
        return analyses
    
    def categorize_content(self, content):
        """Automatically categorize content"""
        categories = []
        content_lower = content.lower()
        
        category_patterns = {
            "robotics": ["robot", "ros", "moveit", "trajectory", "kinematics"],
            "hardware": ["stm32", "arduino", "mcu", "sensor", "driver", "tmc"],
            "software": ["python", "c++", "javascript", "algorithm", "api"],
            "analysis": ["analysis", "comparison", "evaluation", "study"],
            "documentation": ["guide", "manual", "tutorial", "installation"],
            "ai_ml": ["machine learning", "ai", "neural", "model"],
            "engineering": ["design", "cad", "manufacturing", "specifications"],
            "business": ["strategy", "market", "financial", "business"],
            "workflow": ["workflow", "automation", "process", "system"]
        }
        
        for category, keywords in category_patterns.items():
            if any(keyword in content_lower for keyword in keywords):
                categories.append(category)
        
        return categories if categories else ["general"]
    
    def assess_priority(self, content, file_path):
        """Assess integration priority"""
        if any(term in content.lower() for term in ["critical", "urgent", "important"]):
            return "high"
        elif any(term in file_path.lower() for term in ["active", "current"]):
            return "high"
        elif any(term in content.lower() for term in ["draft", "todo"]):
            return "low"
        return "medium"
    
    def update_master_index(self, analyses):
        """Update the master knowledge index"""
        print("📊 Updating master index...")
        
        try:
            # Update statistics in master index
            total_files = len(self.system_state["file_checksums"])
            knowledge_docs = len([f for f in self.system_state["file_checksums"] 
                                if f.endswith('.md')])
            
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M UTC')
            
            # Add activity to log
            activity = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "action": "index_update",
                "files_processed": len(analyses),
                "total_files": total_files,
                "status": "completed"
            }
            self.activity_log["activities"].append(activity)
            self.activity_log["total_operations"] += 1
            
            print(f"✅ Master index updated - {total_files} files tracked")
            
        except Exception as e:
            print(f"⚠️ Error updating master index: {e}")
    
    # === MAINTENANCE OPERATIONS ===
    
    def run_system_maintenance(self, level="daily"):
        """Run system maintenance tasks"""
        print(f"🔧 Running {level} maintenance...")
        
        maintenance_tasks = []
        
        # Git status check
        try:
            os.chdir(self.apm_root)
            result = subprocess.run(['git', 'status', '--porcelain'], 
                                  capture_output=True, text=True)
            if result.stdout.strip():
                maintenance_tasks.append("Git changes detected - may need commit")
            else:
                maintenance_tasks.append("Git status clean")
        except Exception as e:
            maintenance_tasks.append(f"Git check failed: {e}")
        
        # File system health
        try:
            total_size = sum(f.stat().st_size for f in self.apm_root.rglob("*") if f.is_file())
            maintenance_tasks.append(f"System size: {total_size / (1024*1024):.1f} MB")
        except Exception as e:
            maintenance_tasks.append(f"Size check failed: {e}")
        
        # Update maintenance log
        self.system_state["maintenance_tasks"] = maintenance_tasks
        self.system_state["last_maintenance"] = datetime.now(timezone.utc).timestamp()
        
        print(f"✅ Maintenance completed - {len(maintenance_tasks)} tasks checked")
        return maintenance_tasks
    
    def validate_system_health(self):
        """Validate overall system health"""
        print("🏥 Validating system health...")
        
        health_issues = []
        
        # Check critical files exist
        critical_files = [
            "README.md",
            "00_MASTER_KNOWLEDGE_INDEX.md"
        ]
        
        for file_name in critical_files:
            if not (self.apm_root / file_name).exists():
                health_issues.append(f"Missing critical file: {file_name}")
        
        # Check directory structure
        critical_dirs = [
            "Knowledge_Base",
            "PROJECTS",  # Windows repo uses uppercase directory name
            "_ADMIN"
        ]
        
        for dir_name in critical_dirs:
            if not (self.apm_root / dir_name).exists():
                health_issues.append(f"Missing critical directory: {dir_name}")
        
        if health_issues:
            self.system_state["system_health"] = "issues_detected"
            print(f"⚠️ Health issues detected: {len(health_issues)}")
            for issue in health_issues:
                print(f"   • {issue}")
        else:
            self.system_state["system_health"] = "healthy"
            print("✅ System health check passed")
        
        return len(health_issues) == 0
    
    # === GIT OPERATIONS ===
    
    def commit_and_push(self, message="APM System Auto-Update"):
        """Commit and push changes"""
        print("🚀 Committing and pushing changes...")
        
        try:
            os.chdir(self.apm_root)
            
            # Check for changes
            result = subprocess.run(['git', 'status', '--porcelain'], 
                                  capture_output=True, text=True)
            
            if not result.stdout.strip():
                print("📋 No changes to commit")
                return True
            
            # Add and commit
            subprocess.run(['git', 'add', '.'], check=True)
            subprocess.run(['git', 'commit', '-m', message], check=True)
            subprocess.run(['git', 'push', 'origin', 'master'], check=True)
            
            self.system_state["last_push_timestamp"] = datetime.now(timezone.utc).timestamp()
            
            print("✅ Changes committed and pushed")
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"❌ Git operation failed: {e}")
            return False
    
    # === UNIFIED WORKFLOW OPERATIONS ===
    
    def process_knowledge_operation(self, file_path=None, description="", priority="medium"):
        """Process a knowledge operation (main entry point for AI agents)"""
        print("🤖 Processing knowledge operation...")
        
        # Log the operation
        activity = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "action": "knowledge_operation",
            "file_path": file_path,
            "description": description,
            "priority": priority,
            "status": "processing"
        }
        self.activity_log["activities"].append(activity)
        
        # Detect all changes
        new_files, modified_files = self.detect_knowledge_changes()
        all_files = new_files + modified_files
        
        if all_files or file_path:
            # Analyze content
            analyses = self.analyze_content(all_files)
            
            # Update indexes
            self.update_master_index(analyses)
            
            # Validate system
            system_healthy = self.validate_system_health()
            
            if system_healthy:
                # Commit and push
                commit_msg = f"APM Auto-Integration: {datetime.now().strftime('%Y-%m-%d')}"
                if description:
                    commit_msg += f"\n\n{description}"
                
                success = self.commit_and_push(commit_msg)
                
                # Update status
                activity["status"] = "completed" if success else "failed"
                self.system_state["integration_status"] = "completed"
                
                print("✅ Knowledge operation completed successfully")
                return True
            else:
                print("❌ System health issues detected - manual review needed")
                return False
        else:
            print("✅ No changes detected")
            return True
            
        # Always save state
        self.save_system_state()
        self.save_activity_log()
    
    def run_full_workflow(self, mode="auto"):
        """Run complete workflow (for scheduled operations)"""
        print(f"🚀 Running full APM workflow - {mode} mode")
        
        try:
            # System maintenance
            self.run_system_maintenance("daily")
            
            # Process any pending knowledge operations
            success = self.process_knowledge_operation()
            
            # Final health check
            self.validate_system_health()
            
            print("🎉 Full workflow completed successfully")
            return success
            
        except Exception as e:
            print(f"❌ Workflow error: {e}")
            self.system_state["integration_status"] = "error"
            self.save_system_state()
            return False
    
    def get_system_status(self):
        """Get comprehensive system status"""
        status = {
            "system_health": self.system_state.get("system_health", "unknown"),
            "total_files": len(self.system_state.get("file_checksums", {})),
            "last_maintenance": self.system_state.get("last_maintenance", 0),
            "last_push": self.system_state.get("last_push_timestamp", 0),
            "total_operations": self.activity_log.get("total_operations", 0),
            "integration_status": self.system_state.get("integration_status", "idle")
        }
        
        return status

# === AI AGENT INTERFACE FUNCTIONS ===

def process_knowledge(file_path, description="", priority="medium"):
    """Simple interface for AI agents to process knowledge operations"""
    system = APMWorkflowSystem()
    return system.process_knowledge_operation(file_path, description, priority)

def ensure_compliance():
    """Ensure system compliance (simple interface for AI agents)"""
    system = APMWorkflowSystem()
    return system.run_full_workflow("auto")

def get_status():
    """Get system status (simple interface)"""
    system = APMWorkflowSystem()
    return system.get_system_status()

def main():
    """Main CLI interface"""
    parser = argparse.ArgumentParser(description="APM Unified Workflow System")
    parser.add_argument('--mode', choices=['auto', 'manual', 'status', 'maintenance'], 
                       default='auto', help='Operation mode')
    parser.add_argument('--description', default='', help='Operation description')
    
    args = parser.parse_args()
    
    system = APMWorkflowSystem()
    
    if args.mode == 'status':
        status = system.get_system_status()
        print("\n📊 APM System Status:")
        for key, value in status.items():
            print(f"   {key}: {value}")
    
    elif args.mode == 'maintenance':
        system.run_system_maintenance("full")
    
    elif args.mode == 'auto':
        success = system.run_full_workflow("auto")
        exit_code = 0 if success else 1
        exit(exit_code)
    
    elif args.mode == 'manual':
        print("🔧 Manual mode - confirm each step")
        response = input("Proceed with full workflow? (y/N): ")
        if response.lower() == 'y':
            success = system.run_full_workflow("manual")
        else:
            print("⏸️ Cancelled by user")

if __name__ == "__main__":
    main()