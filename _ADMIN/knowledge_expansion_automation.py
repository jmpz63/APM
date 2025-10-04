#!/usr/bin/env python3
"""
APM Knowledge Expansion Automation System
Systematically expands AI knowledge through multiple channels

Usage:
    python3 knowledge_expansion_automation.py --source [pdf|web|github|case-study]
"""

import os
import json
import requests
from pathlib import Path
import datetime

class KnowledgeExpansion:
    def __init__(self):
        self.apm_root = Path("/home/arm1/APM")
        self.knowledge_base = self.apm_root / "Knowledge_Base"
        self.ensure_directories()
        
    def ensure_directories(self):
        """Create knowledge expansion directories"""
        directories = [
            "References/PDFs",
            "References/Research_Papers", 
            "References/Technical_Standards",
            "Case_Studies/Active_Projects",
            "Case_Studies/Completed_Projects",
            "Learning_Materials/Code_Examples",
            "Learning_Materials/Best_Practices",
            "Web_Resources/Scraped_Content",
            "Web_Resources/API_Data"
        ]
        
        for directory in directories:
            (self.knowledge_base / directory).mkdir(parents=True, exist_ok=True)
            
    def scan_for_pdfs(self):
        """Scan and catalog PDF resources"""
        print("📚 Scanning for PDF resources...")
        pdf_dir = self.knowledge_base / "References" / "PDFs"
        
        pdfs_found = list(pdf_dir.glob("*.pdf"))
        
        if pdfs_found:
            print(f"📊 Found {len(pdfs_found)} PDFs to process")
            
            # Create PDF catalog
            catalog = {
                "scan_date": datetime.datetime.now().isoformat(),
                "total_pdfs": len(pdfs_found),
                "pdfs": []
            }
            
            for pdf in pdfs_found:
                catalog["pdfs"].append({
                    "filename": pdf.name,
                    "path": str(pdf),
                    "size": pdf.stat().st_size,
                    "modified": datetime.datetime.fromtimestamp(pdf.stat().st_mtime).isoformat()
                })
            
            # Save catalog
            with open(pdf_dir / "pdf_catalog.json", 'w') as f:
                json.dump(catalog, f, indent=2)
                
            print("✅ PDF catalog created")
            return catalog
        else:
            print("ℹ️  No PDFs found. Place PDFs in APM/Knowledge_Base/References/PDFs/")
            return None
            
    def extract_project_knowledge(self):
        """Extract knowledge from existing projects"""
        print("🔬 Extracting knowledge from active projects...")
        
        projects_dir = self.apm_root / "Projects" / "Active"
        case_studies_dir = self.knowledge_base / "Case_Studies" / "Active_Projects"
        
        knowledge_extracted = []
        
        for project_path in projects_dir.iterdir():
            if project_path.is_dir():
                print(f"   📁 Analyzing {project_path.name}")
                
                # Find key files with knowledge
                knowledge_files = []
                for ext in ['*.md', '*.py', '*.cpp', '*.h', '*.txt', '*.cfg']:
                    knowledge_files.extend(project_path.rglob(ext))
                
                if knowledge_files:
                    case_study = {
                        "project_name": project_path.name,
                        "extraction_date": datetime.datetime.now().isoformat(),
                        "files_analyzed": len(knowledge_files),
                        "knowledge_files": [str(f.relative_to(project_path)) for f in knowledge_files[:10]]  # Top 10
                    }
                    
                    # Create case study file
                    case_study_file = case_studies_dir / f"{project_path.name}_knowledge_extraction.json"
                    with open(case_study_file, 'w') as f:
                        json.dump(case_study, f, indent=2)
                    
                    knowledge_extracted.append(case_study)
        
        print(f"✅ Extracted knowledge from {len(knowledge_extracted)} projects")
        return knowledge_extracted
        
    def github_knowledge_mining(self):
        """Mine knowledge from relevant GitHub repositories"""
        print("🌐 Mining GitHub for relevant knowledge...")
        
        # Relevant repositories for your domains
        target_repos = [
            "ros-planning/moveit2",
            "Klipper3d/klipper", 
            "mainsail-crew/mainsail",
            "stm32duino/Arduino_Core_STM32",
            "micro-ROS/micro_ros_arduino"
        ]
        
        github_data = []
        
        for repo in target_repos:
            try:
                # GitHub API call (no auth needed for public repos)
                response = requests.get(f"https://api.github.com/repos/{repo}")
                
                if response.status_code == 200:
                    repo_data = response.json()
                    
                    knowledge_item = {
                        "repo_name": repo,
                        "description": repo_data.get("description", ""),
                        "language": repo_data.get("language", ""),
                        "stars": repo_data.get("stargazers_count", 0),
                        "last_updated": repo_data.get("updated_at", ""),
                        "topics": repo_data.get("topics", []),
                        "clone_url": repo_data.get("clone_url", "")
                    }
                    
                    github_data.append(knowledge_item)
                    print(f"   ✅ {repo} - {knowledge_item['stars']} stars")
                    
            except Exception as e:
                print(f"   ❌ Failed to fetch {repo}: {e}")
        
        # Save GitHub knowledge
        github_file = self.knowledge_base / "Web_Resources" / "github_knowledge.json"
        with open(github_file, 'w') as f:
            json.dump(github_data, f, indent=2)
            
        print(f"✅ Mined knowledge from {len(github_data)} GitHub repositories")
        return github_data
        
    def create_knowledge_index(self):
        """Create comprehensive knowledge index"""
        print("📊 Creating comprehensive knowledge index...")
        
        # Scan all knowledge sources
        pdf_catalog = self.scan_for_pdfs()
        project_knowledge = self.extract_project_knowledge()
        github_knowledge = self.github_knowledge_mining()
        
        # Create master knowledge index
        master_index = {
            "created": datetime.datetime.now().isoformat(),
            "knowledge_sources": {
                "pdfs": len(pdf_catalog["pdfs"]) if pdf_catalog else 0,
                "case_studies": len(project_knowledge),
                "github_repos": len(github_knowledge),
                "frameworks": 8  # Your advanced frameworks
            },
            "expansion_opportunities": [
                "Add technical standards PDFs (ISO, ANSI)",
                "Document current project procedures", 
                "Clone and analyze relevant GitHub repos",
                "Create troubleshooting case studies",
                "Add equipment manuals and datasheets"
            ],
            "recommended_next_steps": [
                "Place PDFs in Knowledge_Base/References/PDFs/",
                "Document recent project successes and failures",
                "Run web scraping on technical forums",
                "Create detailed configuration guides"
            ]
        }
        
        # Save master index
        index_file = self.knowledge_base / "KNOWLEDGE_EXPANSION_INDEX.md"
        
        content = f"""# APM Knowledge Expansion Index
*Generated: {master_index['created']}*

## 📊 **Current Knowledge Sources**

### **📚 Documents & References**
- **PDFs**: {master_index['knowledge_sources']['pdfs']} files
- **Case Studies**: {master_index['knowledge_sources']['case_studies']} projects analyzed
- **GitHub Repositories**: {master_index['knowledge_sources']['github_repos']} repos indexed
- **Advanced Frameworks**: {master_index['knowledge_sources']['frameworks']} comprehensive systems

### **🚀 Knowledge Expansion Opportunities**
"""
        
        for opportunity in master_index['expansion_opportunities']:
            content += f"- {opportunity}\n"
            
        content += f"""

### **📋 Recommended Next Steps**
"""
        
        for step in master_index['recommended_next_steps']:
            content += f"- {step}\n"
            
        content += f"""

## 🎯 **How to Add Knowledge**

### **📚 PDFs & Documents**
```bash
# Place files here:
/home/arm1/APM/Knowledge_Base/References/PDFs/
/home/arm1/APM/Knowledge_Base/References/Research_Papers/
/home/arm1/APM/Knowledge_Base/References/Technical_Standards/
```

### **🔬 Project Case Studies**  
```bash
# Document projects here:
/home/arm1/APM/Knowledge_Base/Case_Studies/Active_Projects/
/home/arm1/APM/Knowledge_Base/Case_Studies/Completed_Projects/
```

### **💻 Code Examples**
```bash  
# Add code samples here:
/home/arm1/APM/Knowledge_Base/Learning_Materials/Code_Examples/
/home/arm1/APM/Knowledge_Base/Learning_Materials/Best_Practices/
```

### **🌐 Web Resources**
```bash
# Scraped content goes here:
/home/arm1/APM/Knowledge_Base/Web_Resources/Scraped_Content/
/home/arm1/APM/Knowledge_Base/Web_Resources/API_Data/
```

---

**Run knowledge expansion:** `python3 /home/arm1/APM/_ADMIN/knowledge_expansion_automation.py`
"""
        
        with open(index_file, 'w') as f:
            f.write(content)
            
        print("✅ Knowledge expansion index created")
        return master_index

def main():
    """Main knowledge expansion function"""
    expansion = KnowledgeExpansion()
    
    print("🧠 APM Knowledge Expansion System")
    print("==================================")
    
    # Create comprehensive knowledge index
    index = expansion.create_knowledge_index()
    
    print("\n🎯 Knowledge expansion complete!")
    print("📋 See: APM/Knowledge_Base/KNOWLEDGE_EXPANSION_INDEX.md")
    
if __name__ == "__main__":
    main()