#!/usr/bin/env python3
"""
APM Knowledge Search Tool
Fast search across the entire APM knowledge base using the indexed database
"""

import json
import sys
import re
from pathlib import Path

class APMKnowledgeSearch:
    def __init__(self, apm_root="/home/arm1/APM"):
        self.apm_root = Path(apm_root)
        self.db_path = self.apm_root / "_ADMIN" / "knowledge_database.json"
        self.knowledge_db = self.load_knowledge_db()
    
    def load_knowledge_db(self):
        """Load the knowledge database"""
        if not self.db_path.exists():
            print("❌ Knowledge database not found. Run knowledge_indexer.py first.")
            return {}
        
        with open(self.db_path, 'r') as f:
            return json.load(f)
    
    def search(self, query, category=None, limit=10):
        """Search for knowledge matching the query"""
        query_lower = query.lower()
        results = []
        
        knowledge_data = self.knowledge_db.get('knowledge_db', {})
        
        for cat, items in knowledge_data.items():
            if category and cat != category:
                continue
                
            for item in items:
                score = self.calculate_relevance_score(item, query_lower)
                if score > 0:
                    results.append((score, cat, item))
        
        # Sort by relevance score (highest first)
        results.sort(key=lambda x: x[0], reverse=True)
        
        return results[:limit]
    
    def calculate_relevance_score(self, item, query_lower):
        """Calculate how relevant an item is to the search query"""
        score = 0
        
        # Check filename match (high priority)
        if query_lower in item['name'].lower():
            score += 10
        
        # Check path match (medium priority) 
        if query_lower in item['path'].lower():
            score += 5
        
        # Check purpose/doc_type match (high priority)
        purpose = item.get('purpose', item.get('doc_type', '')).lower()
        if query_lower in purpose:
            score += 8
        
        # Check topics match (medium priority)
        topics = item.get('topics', [])
        for topic in topics:
            if query_lower in topic.lower():
                score += 6
        
        # Check features match (for tools)
        features = item.get('features', [])
        for feature in features:
            if query_lower in feature.lower():
                score += 4
        
        # Partial word matching
        words = query_lower.split()
        for word in words:
            if len(word) > 2:  # Skip short words
                text_to_search = f"{item['name']} {purpose} {' '.join(topics)} {' '.join(features)}".lower()
                if word in text_to_search:
                    score += 2
        
        return score
    
    def format_results(self, results):
        """Format search results for display"""
        if not results:
            return "❌ No results found."
        
        output = f"🔍 Found {len(results)} results:\n\n"
        
        for i, (score, category, item) in enumerate(results, 1):
            name = item['name']
            path = item['path']
            purpose = item.get('purpose', item.get('doc_type', 'N/A'))
            topics = item.get('topics', [])
            
            output += f"{i:2d}. 📄 {name}\n"
            output += f"     📁 {path}\n"
            output += f"     🎯 {purpose}\n"
            
            if topics:
                output += f"     🏷️  Tags: {', '.join(topics[:5])}\n"
            
            output += f"     📊 Relevance: {score}/10\n\n"
        
        return output
    
    def search_by_category(self, category=None):
        """List all items in a specific category"""
        knowledge_data = self.knowledge_db.get('knowledge_db', {})
        
        if category:
            if category in knowledge_data:
                return knowledge_data[category]
            else:
                return []
        else:
            # Return all categories
            return list(knowledge_data.keys())
    
    def get_statistics(self):
        """Get knowledge base statistics"""
        stats = self.knowledge_db.get('scan_stats', {})
        knowledge_data = self.knowledge_db.get('knowledge_db', {})
        
        output = "📊 APM Knowledge Base Statistics:\n\n"
        output += f"📄 Total Files: {stats.get('total_files', 0)}\n"
        output += f"📚 Knowledge Docs: {stats.get('knowledge_files', 0)}\n" 
        output += f"🛠️  Tools & Scripts: {stats.get('tools', 0)}\n"
        output += f"⚙️  Config Files: {stats.get('configs', 0)}\n"
        output += f"🕐 Last Scan: {stats.get('last_scan', 'Unknown')}\n\n"
        
        output += "📋 Categories:\n"
        for category, items in knowledge_data.items():
            output += f"   {category}: {len(items)} files\n"
        
        return output

def main():
    if len(sys.argv) < 2:
        print("""
🔍 APM Knowledge Search Tool

Usage:
  python3 knowledge_search.py <query>              # Search for knowledge
  python3 knowledge_search.py --stats              # Show statistics  
  python3 knowledge_search.py --categories         # List categories
  python3 knowledge_search.py --category <cat>     # List items in category

Examples:
  python3 knowledge_search.py "ROS 2"              # Find ROS 2 related knowledge
  python3 knowledge_search.py "chart"              # Find visualization tools
  python3 knowledge_search.py "safety"             # Find safety documentation
  python3 knowledge_search.py --category tools     # List all tools
        """)
        return
    
    searcher = APMKnowledgeSearch()
    
    if sys.argv[1] == "--stats":
        print(searcher.get_statistics())
    elif sys.argv[1] == "--categories":
        categories = searcher.search_by_category()
        print("📋 Available Categories:")
        for cat in sorted(categories):
            print(f"   - {cat}")
    elif sys.argv[1] == "--category" and len(sys.argv) > 2:
        category = sys.argv[2]
        items = searcher.search_by_category(category)
        if items:
            print(f"📁 Items in '{category}' category:")
            for item in items[:20]:  # Limit to 20 items
                print(f"   📄 {item['name']} - {item.get('purpose', 'N/A')}")
        else:
            print(f"❌ Category '{category}' not found or empty")
    else:
        query = " ".join(sys.argv[1:])
        results = searcher.search(query, limit=15)
        print(searcher.format_results(results))

if __name__ == "__main__":
    main()