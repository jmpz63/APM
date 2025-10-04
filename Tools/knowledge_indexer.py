#!/usr/bin/env python3
"""
APM Knowledge Base Auto-Indexer
Automatically scans and catalogs all knowledge in the APM workspace
Updates the Master Knowledge Index with current content
"""

import os
import json
import re
from datetime import datetime
from pathlib import Path

class APMKnowledgeIndexer:
    def __init__(self, apm_root="/home/arm1/APM"):
        self.apm_root = Path(apm_root)
        self.knowledge_db = {}
        self.scan_stats = {
            'total_files': 0,
            'knowledge_files': 0,
            'tools': 0,
            'configs': 0,
            'last_scan': datetime.now().isoformat()
        }
        
    def scan_knowledge_base(self):
        """Comprehensive scan of APM knowledge base"""
        print("🔍 Scanning APM Knowledge Base...")
        
        # Define knowledge file patterns
        knowledge_patterns = {
            'documentation': ['*.md', '*.txt', '*.rst'],
            'tools': ['*.py', '*.sh', '*.bash'],  
            'configs': ['*.yaml', '*.yml', '*.json', '*.cfg', '*.ini'],
            'cad': ['*.scad', '*.stl', '*.step', '*.stp'],
            'data': ['*.csv', '*.xlsx', '*.db']
        }
        
        for root, dirs, files in os.walk(self.apm_root):
            # Skip hidden and cache directories
            dirs[:] = [d for d in dirs if not d.startswith('.') and d != '__pycache__']
            
            for file in files:
                if file.startswith('.'):
                    continue
                    
                file_path = Path(root) / file
                relative_path = file_path.relative_to(self.apm_root)
                
                self.scan_stats['total_files'] += 1
                
                # Categorize file
                category = self.categorize_file(file_path)
                if category:
                    self.add_to_knowledge_db(file_path, relative_path, category)
    
    def categorize_file(self, file_path):
        """Determine the category of a knowledge file"""
        suffix = file_path.suffix.lower()
        name = file_path.name.lower()
        
        # Documentation files
        if suffix in ['.md', '.txt', '.rst']:
            return self.analyze_documentation(file_path)
        
        # Tools and scripts
        elif suffix in ['.py', '.sh', '.bash']:
            return self.analyze_tool(file_path)
            
        # Configuration files  
        elif suffix in ['.yaml', '.yml', '.json', '.cfg', '.ini']:
            return self.analyze_config(file_path)
            
        # CAD and design files
        elif suffix in ['.scad', '.stl', '.step', '.stp']:
            return self.analyze_cad_file(file_path)
            
        return None
    
    def analyze_documentation(self, file_path):
        """Analyze documentation file to determine its purpose and content"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read(2000)  # Read first 2KB for analysis
                
            # Determine document type and topics
            doc_type = self.classify_document_type(content, file_path.name)
            topics = self.extract_topics(content)
            
            self.scan_stats['knowledge_files'] += 1
            
            return {
                'type': 'documentation',
                'doc_type': doc_type,
                'topics': topics,
                'size': file_path.stat().st_size,
                'modified': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
            }
        except Exception as e:
            return None
    
    def analyze_tool(self, file_path):
        """Analyze tool/script file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read(1000)  # Read first 1KB
                
            # Extract purpose from docstring or comments
            purpose = self.extract_tool_purpose(content)
            features = self.extract_tool_features(content)
            
            self.scan_stats['tools'] += 1
            
            return {
                'type': 'tool',
                'purpose': purpose,
                'features': features,
                'executable': os.access(file_path, os.X_OK),
                'language': 'python' if file_path.suffix == '.py' else 'bash',
                'size': file_path.stat().st_size,
                'modified': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
            }
        except Exception:
            return None
    
    def analyze_config(self, file_path):
        """Analyze configuration file"""
        self.scan_stats['configs'] += 1
        
        return {
            'type': 'configuration',
            'format': file_path.suffix[1:],  # Remove the dot
            'size': file_path.stat().st_size,
            'modified': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
        }
    
    def analyze_cad_file(self, file_path):
        """Analyze CAD/design file"""
        return {
            'type': 'cad',
            'format': file_path.suffix[1:].upper(),
            'size': file_path.stat().st_size,
            'modified': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
        }
    
    def classify_document_type(self, content, filename):
        """Classify the type of documentation"""
        content_lower = content.lower()
        filename_lower = filename.lower()
        
        # Check for specific document types
        if 'lesson' in filename_lower or 'lesson' in content_lower:
            return 'lessons_learned'
        elif 'onboard' in filename_lower or 'onboard' in content_lower:
            return 'onboarding_guide'
        elif 'api' in content_lower or 'endpoint' in content_lower:
            return 'api_documentation'
        elif 'install' in content_lower or 'setup' in content_lower:
            return 'installation_guide'
        elif 'troubleshoot' in content_lower or 'debug' in content_lower:
            return 'troubleshooting_guide'
        elif 'architecture' in content_lower or 'design' in content_lower:
            return 'architecture_documentation'
        elif filename_lower in ['readme.md', 'readme.txt']:
            return 'project_readme'
        elif 'index' in filename_lower:
            return 'knowledge_index'
        else:
            return 'technical_documentation'
    
    def extract_topics(self, content):
        """Extract key topics from document content"""
        # Look for common technical terms
        topics = set()
        
        topic_patterns = {
            'ros2': r'\bros\s*2\b|ros2',
            'klipper': r'\bklipper\b',
            'moveit': r'\bmoveit\b',
            'safety': r'\bsafety\b|iso\s*13849',
            'manufacturing': r'\bmanufacturing\b|\bcad\b|\b3d.*print',
            'automation': r'\bautomation\b|\brobot',
            'hardware': r'\bhardware\b|\bcontrol\b|\bstepper',
            'software': r'\bsoftware\b|\bpython\b|\bscript',
            'visualization': r'\bvisualization\b|\bchart\b|\bgraph',
            'integration': r'\bintegration\b|\bpipeline\b'
        }
        
        content_lower = content.lower()
        for topic, pattern in topic_patterns.items():
            if re.search(pattern, content_lower):
                topics.add(topic)
        
        return list(topics)
    
    def extract_tool_purpose(self, content):
        """Extract tool purpose from docstring or comments"""
        # Look for docstring
        docstring_match = re.search(r'"""(.*?)"""', content, re.DOTALL)
        if docstring_match:
            return docstring_match.group(1).strip().split('\n')[0]
        
        # Look for initial comments
        comment_match = re.search(r'^#\s*(.*?)$', content, re.MULTILINE)
        if comment_match:
            return comment_match.group(1).strip()
        
        return "Unknown purpose"
    
    def extract_tool_features(self, content):
        """Extract key features from tool code"""
        features = []
        
        # Look for common patterns
        if 'matplotlib' in content or 'plt.' in content:
            features.append('data_visualization')
        if 'http.server' in content or 'HTTPServer' in content:
            features.append('web_server')
        if 'subprocess' in content:
            features.append('system_integration')
        if 'argparse' in content:
            features.append('command_line_interface')
        if 'json' in content:
            features.append('data_processing')
        
        return features
    
    def add_to_knowledge_db(self, file_path, relative_path, metadata):
        """Add file to knowledge database"""
        category = metadata['type']
        
        if category not in self.knowledge_db:
            self.knowledge_db[category] = []
        
        self.knowledge_db[category].append({
            'path': str(relative_path),
            'full_path': str(file_path),
            'name': file_path.name,
            **metadata
        })
    
    def generate_master_index_update(self):
        """Generate updated content for master knowledge index"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Generate statistics summary
        stats_summary = f"""
## 📊 **Knowledge Base Statistics** (Auto-Generated)

*Last Scan: {timestamp}*

### **Content Overview**
- **Total Files Scanned:** {self.scan_stats['total_files']}
- **Knowledge Documents:** {self.scan_stats['knowledge_files']}
- **Tools & Scripts:** {self.scan_stats['tools']}
- **Configuration Files:** {self.scan_stats['configs']}

### **Knowledge Distribution**
"""
        
        # Add knowledge category breakdown
        for category, items in self.knowledge_db.items():
            stats_summary += f"- **{category.title()}:** {len(items)} files\n"
        
        # Generate detailed catalog
        catalog_update = "\n## 🔍 **Auto-Generated Knowledge Catalog**\n\n"
        
        for category, items in self.knowledge_db.items():
            if not items:
                continue
                
            catalog_update += f"### **{category.title().replace('_', ' ')}**\n\n"
            catalog_update += "| File | Purpose | Topics | Last Modified |\n"
            catalog_update += "|------|---------|--------|---------------|\n"
            
            for item in sorted(items, key=lambda x: x['name'])[:10]:  # Top 10 per category
                name = item['name']
                path = item['path']
                purpose = item.get('purpose', item.get('doc_type', 'N/A'))[:50]
                topics = ', '.join(item.get('topics', [])[:3])  # First 3 topics
                modified = item['modified'][:10]  # Date only
                
                catalog_update += f"| [{name}]({path}) | {purpose} | {topics} | {modified} |\n"
            
            catalog_update += "\n"
        
        return stats_summary + catalog_update
    
    def save_knowledge_database(self):
        """Save the knowledge database to JSON for future reference"""
        db_path = self.apm_root / "_ADMIN" / "knowledge_database.json"
        os.makedirs(db_path.parent, exist_ok=True)
        
        with open(db_path, 'w') as f:
            json.dump({
                'scan_stats': self.scan_stats,
                'knowledge_db': self.knowledge_db,
                'generated': datetime.now().isoformat()
            }, f, indent=2)
        
        print(f"💾 Knowledge database saved to: {db_path}")
        return db_path
    
    def update_master_index(self):
        """Update the master knowledge index file"""
        master_index_path = self.apm_root / "00_MASTER_KNOWLEDGE_INDEX.md"
        
        if not master_index_path.exists():
            print("❌ Master index file not found")
            return
        
        # Read current index
        with open(master_index_path, 'r') as f:
            current_content = f.read()
        
        # Generate new statistics section
        new_stats = self.generate_master_index_update()
        
        # Find insertion point (after the quick navigation hub)
        insert_marker = "---\n\n## 📚 **Knowledge Base Catalog**"
        
        if insert_marker in current_content:
            # Insert before the existing catalog
            parts = current_content.split(insert_marker, 1)
            updated_content = parts[0] + "\n" + new_stats + "\n" + insert_marker + parts[1]
        else:
            # Append to end
            updated_content = current_content + "\n" + new_stats
        
        # Write updated content
        with open(master_index_path, 'w') as f:
            f.write(updated_content)
        
        print(f"✅ Master knowledge index updated: {master_index_path}")

def main():
    print("🎯 APM Knowledge Base Auto-Indexer Starting...")
    
    indexer = APMKnowledgeIndexer()
    
    # Scan the knowledge base
    indexer.scan_knowledge_base()
    
    # Generate reports
    db_path = indexer.save_knowledge_database()
    indexer.update_master_index()
    
    # Print summary
    print(f"\n📈 Scan Summary:")
    print(f"   📄 Total files: {indexer.scan_stats['total_files']}")
    print(f"   📚 Knowledge documents: {indexer.scan_stats['knowledge_files']}")
    print(f"   🛠️  Tools & scripts: {indexer.scan_stats['tools']}")
    print(f"   ⚙️  Configuration files: {indexer.scan_stats['configs']}")
    
    print(f"\n✅ Knowledge indexing complete!")
    print(f"📊 View Master Index: ~/APM/00_MASTER_KNOWLEDGE_INDEX.md")
    print(f"💾 Raw database: {db_path}")

if __name__ == "__main__":
    main()