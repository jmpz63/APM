#!/usr/bin/env python3
"""
Interactive Drill-Down Folder Explorer
Creates dynamic bar charts for any folder path with web interface
"""

import os
import json
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import threading
import time

class FolderAnalyzer:
    def __init__(self):
        self.chart_dir = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/charts"
        os.makedirs(self.chart_dir, exist_ok=True)
        
    def get_folder_data(self, folder_path):
        """Analyze folder and return size/count data for subfolders"""
        if not os.path.exists(folder_path) or not os.path.isdir(folder_path):
            return {"error": f"Path not found: {folder_path}"}
        
        folder_data = {}
        try:
            items = os.listdir(folder_path)
            for item in items:
                if item.startswith('.'):
                    continue
                    
                item_path = os.path.join(folder_path, item)
                if os.path.isdir(item_path):
                    size_mb = self.get_folder_size_mb(item_path)
                    file_count = self.get_file_count(item_path)
                    folder_data[item] = {
                        'size_mb': size_mb,
                        'file_count': file_count,
                        'path': item_path
                    }
        except PermissionError:
            return {"error": f"Permission denied: {folder_path}"}
        
        return folder_data
    
    def get_folder_size_mb(self, folder_path):
        """Get folder size in MB"""
        try:
            result = subprocess.run(['du', '-sm', folder_path], 
                                  capture_output=True, text=True, timeout=5)
            return int(result.stdout.split()[0]) if result.returncode == 0 else 0
        except:
            return 0
    
    def get_file_count(self, folder_path):
        """Get file count in folder"""
        try:
            result = subprocess.run(['find', folder_path, '-type', 'f'], 
                                  capture_output=True, text=True, timeout=5)
            return len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
        except:
            return 0
    
    def create_chart(self, folder_path, folder_data):
        """Create bar chart for specific folder"""
        if not folder_data or "error" in folder_data:
            return None
        
        # Prepare data
        folders = list(folder_data.keys())
        sizes = [folder_data[f]['size_mb'] for f in folders]
        counts = [folder_data[f]['file_count'] for f in folders]
        
        if not folders:
            return None
        
        # Sort by size
        sorted_data = sorted(zip(folders, sizes, counts), key=lambda x: x[1], reverse=True)
        folders_sorted = [x[0] for x in sorted_data]
        sizes_sorted = [x[1] for x in sorted_data]
        
        # Create chart
        plt.figure(figsize=(14, 8))
        colors = plt.cm.Set3(range(len(folders_sorted)))
        bars = plt.bar(folders_sorted, sizes_sorted, color=colors, edgecolor='black', linewidth=1)
        
        # Add labels
        for bar, size in zip(bars, sizes_sorted):
            if size > 0:
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(sizes_sorted)*0.02,
                        f'{size} MB', ha='center', va='bottom', fontweight='bold', fontsize=10)
        
        plt.title(f'Folder Analysis: {folder_path}', fontsize=16, fontweight='bold', pad=20)
        plt.ylabel('Size (MB)', fontsize=12, fontweight='bold')
        plt.xlabel('Subdirectories', fontsize=12, fontweight='bold')
        plt.xticks(rotation=45, ha='right')
        plt.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        
        # Save chart
        chart_name = folder_path.replace('/', '_').replace(' ', '_') + '_chart.png'
        chart_path = os.path.join(self.chart_dir, chart_name)
        plt.savefig(chart_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
        
        return chart_name

class DrillDownHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, analyzer=None, **kwargs):
        self.analyzer = analyzer
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        parsed_path = urlparse(self.path)
        
        if parsed_path.path == '/':
            self.serve_main_page()
        elif parsed_path.path == '/analyze':
            self.handle_analyze_request(parsed_path.query)
        elif parsed_path.path.endswith('.png'):
            self.serve_image(parsed_path.path[1:])
        elif parsed_path.path.endswith('.html'):
            self.serve_html(parsed_path.path[1:])
        else:
            self.send_error(404)
    
    def serve_main_page(self):
        """Serve the main interactive page"""
        html_content = self.create_interactive_html()
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(html_content.encode())
    
    def handle_analyze_request(self, query):
        """Handle AJAX request for folder analysis"""
        params = parse_qs(query)
        folder_path = params.get('path', ['/home/arm1/APM'])[0]
        
        folder_data = self.analyzer.get_folder_data(folder_path)
        
        if "error" not in folder_data:
            chart_name = self.analyzer.create_chart(folder_path, folder_data)
            response = {
                'success': True,
                'chart': chart_name,
                'data': folder_data,
                'path': folder_path
            }
        else:
            response = {
                'success': False,
                'error': folder_data['error']
            }
        
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode())
    
    def serve_image(self, filename):
        """Serve chart images"""
        file_path = os.path.join(self.analyzer.chart_dir, filename)
        if os.path.exists(file_path):
            self.send_response(200)
            self.send_header('Content-type', 'image/png')
            self.end_headers()
            with open(file_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)
    
    def serve_html(self, filename):
        """Serve HTML files"""
        file_path = os.path.join(self.analyzer.chart_dir, filename)
        if os.path.exists(file_path):
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open(file_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404)
    
    def create_interactive_html(self):
        """Create the interactive drill-down HTML interface"""
        return '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Interactive Folder Explorer</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            overflow: hidden;
        }
        
        .header {
            background: linear-gradient(135deg, #2E86AB 0%, #A23B72 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }
        
        .header h1 {
            font-size: 2.5em;
            font-weight: 300;
            margin-bottom: 10px;
        }
        
        .breadcrumb {
            background: #f8f9fa;
            padding: 20px;
            border-bottom: 1px solid #dee2e6;
        }
        
        .breadcrumb-item {
            display: inline-block;
            color: #2E86AB;
            cursor: pointer;
            text-decoration: underline;
            margin-right: 10px;
        }
        
        .breadcrumb-item:hover {
            color: #A23B72;
        }
        
        .breadcrumb-separator {
            margin: 0 5px;
            color: #6c757d;
        }
        
        .controls {
            padding: 20px;
            background: #f8f9fa;
            display: flex;
            gap: 20px;
            align-items: center;
        }
        
        .path-input {
            flex: 1;
            padding: 10px;
            border: 2px solid #dee2e6;
            border-radius: 5px;
            font-size: 16px;
        }
        
        .analyze-btn {
            padding: 10px 20px;
            background: #2E86AB;
            color: white;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
        }
        
        .analyze-btn:hover {
            background: #A23B72;
        }
        
        .loading {
            text-align: center;
            padding: 40px;
            font-size: 18px;
            color: #6c757d;
        }
        
        .chart-section {
            padding: 30px;
        }
        
        .chart-container {
            text-align: center;
            margin-bottom: 30px;
        }
        
        .chart-container img {
            max-width: 100%;
            height: auto;
            border-radius: 10px;
            box-shadow: 0 5px 15px rgba(0,0,0,0.1);
        }
        
        .folder-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-top: 30px;
        }
        
        .folder-card {
            background: white;
            border: 2px solid #dee2e6;
            border-radius: 10px;
            padding: 20px;
            cursor: pointer;
            transition: all 0.3s;
        }
        
        .folder-card:hover {
            border-color: #2E86AB;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.1);
        }
        
        .folder-name {
            font-size: 18px;
            font-weight: bold;
            color: #2E86AB;
            margin-bottom: 10px;
        }
        
        .folder-stats {
            color: #6c757d;
            font-size: 14px;
        }
        
        .error {
            background: #f8d7da;
            color: #721c24;
            padding: 20px;
            border-radius: 5px;
            margin: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🔍 Interactive Folder Explorer</h1>
            <p>Click any folder to drill down and see its contents</p>
        </div>
        
        <div class="breadcrumb" id="breadcrumb">
            <span class="breadcrumb-item" onclick="analyzePath('/home/arm1')">/home/arm1</span>
        </div>
        
        <div class="controls">
            <input type="text" id="pathInput" class="path-input" value="/home/arm1/APM" placeholder="Enter folder path...">
            <button class="analyze-btn" onclick="analyzeCurrentPath()">🔍 Analyze</button>
        </div>
        
        <div id="content">
            <div class="loading">
                Click "Analyze" to start exploring folders...
            </div>
        </div>
    </div>
    
    <script>
        let currentPath = '/home/arm1/APM';
        let pathHistory = ['/home/arm1'];
        
        function updateBreadcrumb() {
            const breadcrumb = document.getElementById('breadcrumb');
            breadcrumb.innerHTML = pathHistory.map((path, index) => {
                const displayName = path === '/home/arm1' ? '/home/arm1' : path.split('/').pop();
                return `<span class="breadcrumb-item" onclick="analyzePath('${path}')">${displayName}</span>`;
            }).join('<span class="breadcrumb-separator">→</span>');
        }
        
        function analyzePath(path) {
            currentPath = path;
            document.getElementById('pathInput').value = path;
            
            // Update history
            const pathIndex = pathHistory.indexOf(path);
            if (pathIndex !== -1) {
                pathHistory = pathHistory.slice(0, pathIndex + 1);
            } else {
                pathHistory.push(path);
            }
            
            updateBreadcrumb();
            
            document.getElementById('content').innerHTML = '<div class="loading">🔄 Analyzing folder contents...</div>';
            
            fetch(`/analyze?path=${encodeURIComponent(path)}`)
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        displayResults(data);
                    } else {
                        document.getElementById('content').innerHTML = `<div class="error">❌ ${data.error}</div>`;
                    }
                })
                .catch(error => {
                    document.getElementById('content').innerHTML = `<div class="error">❌ Network error: ${error}</div>`;
                });
        }
        
        function analyzeCurrentPath() {
            const path = document.getElementById('pathInput').value.trim();
            if (path) {
                analyzePath(path);
            }
        }
        
        function displayResults(data) {
            const folders = Object.keys(data.data);
            const totalSize = Object.values(data.data).reduce((sum, folder) => sum + folder.size_mb, 0);
            const totalFiles = Object.values(data.data).reduce((sum, folder) => sum + folder.file_count, 0);
            
            let html = `
                <div class="chart-section">
                    <div class="chart-container">
                        <img src="${data.chart}" alt="Folder Chart">
                        <p><strong>Total: ${totalSize} MB, ${totalFiles} files in ${folders.length} subdirectories</strong></p>
                    </div>
                    
                    <div class="folder-grid">
            `;
            
            // Sort folders by size
            const sortedFolders = folders.sort((a, b) => data.data[b].size_mb - data.data[a].size_mb);
            
            sortedFolders.forEach(folder => {
                const folderData = data.data[folder];
                const folderPath = folderData.path;
                
                html += `
                    <div class="folder-card" onclick="analyzePath('${folderPath}')">
                        <div class="folder-name">📁 ${folder}</div>
                        <div class="folder-stats">
                            Size: ${folderData.size_mb} MB<br>
                            Files: ${folderData.file_count}<br>
                            Click to explore →
                        </div>
                    </div>
                `;
            });
            
            html += '</div></div>';
            document.getElementById('content').innerHTML = html;
        }
        
        // Auto-analyze APM folder on load
        window.onload = function() {
            analyzePath('/home/arm1/APM');
        };
        
        // Enter key support
        document.addEventListener('keypress', function(event) {
            if (event.key === 'Enter' && document.activeElement.id === 'pathInput') {
                analyzeCurrentPath();
            }
        });
    </script>
</body>
</html>'''

def create_handler_class(analyzer):
    """Create handler class with analyzer instance"""
    class Handler(DrillDownHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, analyzer=analyzer, **kwargs)
    return Handler

def main():
    print("🚀 Starting Interactive Drill-Down Folder Explorer...")
    
    analyzer = FolderAnalyzer()
    handler_class = create_handler_class(analyzer)
    
    server = HTTPServer(('localhost', 8081), handler_class)
    
    print("✅ Server running at http://localhost:8081")
    print("🔍 Interactive folder exploration ready!")
    print("📊 Click any folder to drill down and see its bar chart")
    print("\nPress Ctrl+C to stop the server")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")
        server.server_close()

if __name__ == "__main__":
    main()