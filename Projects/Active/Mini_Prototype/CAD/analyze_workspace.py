#!/usr/bin/env python3
"""
Directory Size Visualization Tool for APM Workspace
Creates bar charts showing folder sizes and file counts
"""

import os
import matplotlib.pyplot as plt
import subprocess
import json
from pathlib import Path

class DirectorySizeVisualizer:
    def __init__(self, base_path="/home/arm1"):
        self.base_path = base_path
        self.folder_data = {}
    
    def get_folder_size_and_count(self, folder_path):
        """Get folder size in MB and file count"""
        try:
            # Get size using du command (faster than Python traversal)
            result = subprocess.run(['du', '-sm', folder_path], 
                                  capture_output=True, text=True, timeout=30)
            size_mb = int(result.stdout.split()[0]) if result.returncode == 0 else 0
            
            # Get file count
            result_count = subprocess.run(['find', folder_path, '-type', 'f'], 
                                        capture_output=True, text=True, timeout=30)
            file_count = len(result_count.stdout.strip().split('\n')) if result_count.stdout.strip() else 0
            
            return size_mb, file_count
            
        except (subprocess.TimeoutExpired, ValueError, Exception):
            return 0, 0
    
    def scan_directories(self):
        """Scan all top-level directories and collect size data"""
        print(f"🔍 Scanning directories in {self.base_path}...")
        
        folders = []
        try:
            for item in os.listdir(self.base_path):
                item_path = os.path.join(self.base_path, item)
                if os.path.isdir(item_path) and not item.startswith('.'):
                    folders.append(item)
        except PermissionError:
            print(f"❌ Permission denied accessing {self.base_path}")
            return
        
        total_folders = len(folders)
        for i, folder in enumerate(folders, 1):
            folder_path = os.path.join(self.base_path, folder)
            print(f"📁 Analyzing {folder}... ({i}/{total_folders})")
            
            size_mb, file_count = self.get_folder_size_and_count(folder_path)
            
            self.folder_data[folder] = {
                'size_mb': size_mb,
                'file_count': file_count,
                'path': folder_path
            }
            
            print(f"   └─ {size_mb} MB, {file_count} files")
    
    def create_size_bar_chart(self, top_n=15):
        """Create bar chart showing folder sizes"""
        if not self.folder_data:
            print("❌ No data to visualize. Run scan_directories() first.")
            return
        
        # Sort by size and take top N
        sorted_folders = sorted(self.folder_data.items(), 
                              key=lambda x: x[1]['size_mb'], reverse=True)[:top_n]
        
        if not sorted_folders:
            print("❌ No folders with measurable size found.")
            return
        
        folders = [item[0] for item in sorted_folders]
        sizes = [item[1]['size_mb'] for item in sorted_folders]
        
        # Create color gradient based on size
        colors = plt.cm.viridis([i/len(sizes) for i in range(len(sizes))])
        
        plt.figure(figsize=(14, 8))
        bars = plt.bar(range(len(folders)), sizes, color=colors, alpha=0.8, edgecolor='black')
        
        # Add size labels on bars
        for i, (bar, size) in enumerate(zip(bars, sizes)):
            if size > 0:
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(sizes)*0.01,
                        f'{size} MB', ha='center', va='bottom', fontweight='bold', fontsize=10)
        
        plt.title(f'Directory Sizes - Top {len(folders)} Largest Folders\nWorkspace: {self.base_path}', 
                 fontsize=16, fontweight='bold', pad=20)
        plt.ylabel('Size (MB)', fontsize=12, fontweight='bold')
        plt.xlabel('Directories', fontsize=12, fontweight='bold')
        
        # Format x-axis labels
        plt.xticks(range(len(folders)), folders, rotation=45, ha='right')
        plt.grid(axis='y', alpha=0.3)
        
        # Add total size info
        total_size = sum(sizes)
        plt.text(0.02, 0.98, f'Total Analyzed: {total_size:,} MB', 
                transform=plt.gca().transAxes, fontsize=11, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        plt.tight_layout()
        
        # Save chart
        output_path = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/charts/directory_sizes.png"
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()  # Close instead of show to avoid GUI issues
        
        print(f"✅ Directory size chart saved: {output_path}")
        return output_path
    
    def create_file_count_chart(self, top_n=15):
        """Create bar chart showing file counts per directory"""
        if not self.folder_data:
            return
        
        # Sort by file count
        sorted_folders = sorted(self.folder_data.items(), 
                              key=lambda x: x[1]['file_count'], reverse=True)[:top_n]
        
        folders = [item[0] for item in sorted_folders]
        counts = [item[1]['file_count'] for item in sorted_folders]
        
        # Create different color scheme for file counts
        colors = plt.cm.plasma([i/len(counts) for i in range(len(counts))])
        
        plt.figure(figsize=(14, 8))
        bars = plt.bar(range(len(folders)), counts, color=colors, alpha=0.8, edgecolor='black')
        
        # Add count labels on bars
        for i, (bar, count) in enumerate(zip(bars, counts)):
            if count > 0:
                plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(counts)*0.01,
                        f'{count:,}', ha='center', va='bottom', fontweight='bold', fontsize=10)
        
        plt.title(f'File Counts by Directory - Top {len(folders)} Most Populated Folders\nWorkspace: {self.base_path}', 
                 fontsize=16, fontweight='bold', pad=20)
        plt.ylabel('Number of Files', fontsize=12, fontweight='bold')
        plt.xlabel('Directories', fontsize=12, fontweight='bold')
        
        plt.xticks(range(len(folders)), folders, rotation=45, ha='right')
        plt.grid(axis='y', alpha=0.3)
        
        # Add total file count info
        total_files = sum(counts)
        plt.text(0.02, 0.98, f'Total Files: {total_files:,}', 
                transform=plt.gca().transAxes, fontsize=11,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        plt.tight_layout()
        
        # Save chart
        output_path = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/charts/file_counts.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.show()
        
        print(f"✅ File count chart saved: {output_path}")
        return output_path
    
    def create_combined_dashboard(self):
        """Create a combined dashboard showing both size and file count"""
        if not self.folder_data:
            return
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # Top folders by size
        sorted_by_size = sorted(self.folder_data.items(), 
                               key=lambda x: x[1]['size_mb'], reverse=True)[:10]
        folders_size = [item[0] for item in sorted_by_size]
        sizes = [item[1]['size_mb'] for item in sorted_by_size]
        
        ax1.bar(range(len(folders_size)), sizes, color='skyblue', alpha=0.8)
        ax1.set_title('Top 10 Folders by Size (MB)', fontsize=14, fontweight='bold')
        ax1.set_ylabel('Size (MB)', fontweight='bold')
        ax1.set_xticks(range(len(folders_size)))
        ax1.set_xticklabels(folders_size, rotation=45, ha='right')
        ax1.grid(axis='y', alpha=0.3)
        
        # Add size labels
        for i, size in enumerate(sizes):
            if size > 0:
                ax1.text(i, size + max(sizes)*0.01, f'{size}', ha='center', va='bottom', fontweight='bold')
        
        # Top folders by file count
        sorted_by_count = sorted(self.folder_data.items(), 
                                key=lambda x: x[1]['file_count'], reverse=True)[:10]
        folders_count = [item[0] for item in sorted_by_count]
        counts = [item[1]['file_count'] for item in sorted_by_count]
        
        ax2.bar(range(len(folders_count)), counts, color='lightcoral', alpha=0.8)
        ax2.set_title('Top 10 Folders by File Count', fontsize=14, fontweight='bold')
        ax2.set_ylabel('Number of Files', fontweight='bold')
        ax2.set_xticks(range(len(folders_count)))
        ax2.set_xticklabels(folders_count, rotation=45, ha='right')
        ax2.grid(axis='y', alpha=0.3)
        
        # Add count labels
        for i, count in enumerate(counts):
            if count > 0:
                ax2.text(i, count + max(counts)*0.01, f'{count:,}', ha='center', va='bottom', fontweight='bold')
        
        plt.suptitle(f'Workspace Analysis Dashboard: {self.base_path}', fontsize=18, fontweight='bold')
        plt.tight_layout()
        
        # Save dashboard
        output_path = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/charts/workspace_dashboard.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
        plt.show()
        
        print(f"✅ Workspace dashboard saved: {output_path}")
        return output_path
    
    def print_summary_report(self):
        """Print a text summary of findings"""
        if not self.folder_data:
            return
        
        print("\n" + "="*60)
        print("📊 WORKSPACE ANALYSIS SUMMARY")
        print("="*60)
        
        total_size = sum([data['size_mb'] for data in self.folder_data.values()])
        total_files = sum([data['file_count'] for data in self.folder_data.values()])
        
        print(f"📁 Total Directories Analyzed: {len(self.folder_data)}")
        print(f"💾 Total Space Used: {total_size:,} MB ({total_size/1024:.1f} GB)")
        print(f"📄 Total Files: {total_files:,}")
        
        print(f"\n🔝 TOP 5 LARGEST DIRECTORIES:")
        sorted_by_size = sorted(self.folder_data.items(), 
                               key=lambda x: x[1]['size_mb'], reverse=True)[:5]
        for i, (folder, data) in enumerate(sorted_by_size, 1):
            percentage = (data['size_mb'] / total_size * 100) if total_size > 0 else 0
            print(f"   {i}. {folder}: {data['size_mb']:,} MB ({percentage:.1f}%)")
        
        print(f"\n📊 TOP 5 MOST POPULATED DIRECTORIES:")
        sorted_by_count = sorted(self.folder_data.items(), 
                                key=lambda x: x[1]['file_count'], reverse=True)[:5]
        for i, (folder, data) in enumerate(sorted_by_count, 1):
            percentage = (data['file_count'] / total_files * 100) if total_files > 0 else 0
            print(f"   {i}. {folder}: {data['file_count']:,} files ({percentage:.1f}%)")
        
        print("="*60)

def main():
    print("🎯 APM Workspace Directory Size Analyzer")
    print("="*50)
    
    # Create visualizer
    visualizer = DirectorySizeVisualizer()
    
    # Scan directories
    visualizer.scan_directories()
    
    # Create visualizations
    print(f"\n📈 Generating visualizations...")
    visualizer.create_size_bar_chart()
    visualizer.create_file_count_chart()
    visualizer.create_combined_dashboard()
    
    # Print summary
    visualizer.print_summary_report()
    
    print(f"\n✅ Analysis complete! Charts saved in charts/ directory")

if __name__ == "__main__":
    main()