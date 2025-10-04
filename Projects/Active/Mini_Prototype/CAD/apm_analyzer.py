#!/usr/bin/env python3
"""
APM Folder Size Visualizer - Simple Bar Graph Generator
Creates bar charts showing APM subfolder sizes without GUI popups
"""

import os
import matplotlib.pyplot as plt
import subprocess
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend

def get_folder_size_mb(folder_path):
    """Get folder size in MB using du command"""
    try:
        result = subprocess.run(['du', '-sm', folder_path], 
                              capture_output=True, text=True, timeout=10)
        return int(result.stdout.split()[0]) if result.returncode == 0 else 0
    except:
        return 0

def get_file_count(folder_path):
    """Get number of files in folder"""
    try:
        result = subprocess.run(['find', folder_path, '-type', 'f'], 
                              capture_output=True, text=True, timeout=10)
        return len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
    except:
        return 0

def analyze_apm_folder():
    """Analyze APM folder structure and create bar charts"""
    apm_path = "/home/arm1/APM"
    
    print(f"🔍 Analyzing APM folder structure...")
    
    folder_data = {}
    
    # Scan APM subdirectories
    try:
        for item in os.listdir(apm_path):
            item_path = os.path.join(apm_path, item)
            if os.path.isdir(item_path):
                print(f"📁 Scanning {item}...")
                size_mb = get_folder_size_mb(item_path)
                file_count = get_file_count(item_path)
                folder_data[item] = {'size_mb': size_mb, 'file_count': file_count}
                print(f"   └─ {size_mb} MB, {file_count} files")
    except Exception as e:
        print(f"❌ Error scanning APM folder: {e}")
        return
    
    if not folder_data:
        print("❌ No subfolders found in APM directory")
        return
    
    # Create output directory
    output_dir = "/home/arm1/APM/Projects/Active/Mini_Prototype/CAD/charts"
    os.makedirs(output_dir, exist_ok=True)
    
    # Create size bar chart
    folders = list(folder_data.keys())
    sizes = [folder_data[f]['size_mb'] for f in folders]
    counts = [folder_data[f]['file_count'] for f in folders]
    
    # Sort by size for better visualization
    sorted_data = sorted(zip(folders, sizes, counts), key=lambda x: x[1], reverse=True)
    folders_sorted = [x[0] for x in sorted_data]
    sizes_sorted = [x[1] for x in sorted_data]
    counts_sorted = [x[2] for x in sorted_data]
    
    # Create folder size chart
    plt.figure(figsize=(12, 8))
    colors = plt.cm.Set3(range(len(folders_sorted)))
    bars = plt.bar(folders_sorted, sizes_sorted, color=colors, edgecolor='black', linewidth=1)
    
    # Add size labels on bars
    for bar, size in zip(bars, sizes_sorted):
        if size > 0:
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(sizes_sorted)*0.02,
                    f'{size} MB', ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    plt.title('APM Folder Sizes - Directory Storage Usage', fontsize=16, fontweight='bold', pad=20)
    plt.ylabel('Size (MB)', fontsize=12, fontweight='bold')
    plt.xlabel('APM Subdirectories', fontsize=12, fontweight='bold')
    plt.xticks(rotation=45, ha='right')
    plt.grid(axis='y', alpha=0.3)
    
    # Add total info
    total_size = sum(sizes_sorted)
    plt.text(0.02, 0.98, f'Total APM Size: {total_size} MB', 
            transform=plt.gca().transAxes, fontsize=11, 
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()
    size_chart_path = os.path.join(output_dir, 'apm_folder_sizes.png')
    plt.savefig(size_chart_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()
    
    # Create file count chart
    plt.figure(figsize=(12, 8))
    bars2 = plt.bar(folders_sorted, counts_sorted, color=colors, edgecolor='black', linewidth=1)
    
    # Add count labels on bars
    for bar, count in zip(bars2, counts_sorted):
        if count > 0:
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(counts_sorted)*0.02,
                    f'{count}', ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    plt.title('APM Folder File Counts - Number of Files per Directory', fontsize=16, fontweight='bold', pad=20)
    plt.ylabel('Number of Files', fontsize=12, fontweight='bold')
    plt.xlabel('APM Subdirectories', fontsize=12, fontweight='bold')
    plt.xticks(rotation=45, ha='right')
    plt.grid(axis='y', alpha=0.3)
    
    # Add total info
    total_files = sum(counts_sorted)
    plt.text(0.02, 0.98, f'Total Files: {total_files}', 
            transform=plt.gca().transAxes, fontsize=11,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    count_chart_path = os.path.join(output_dir, 'apm_file_counts.png')
    plt.savefig(count_chart_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()
    
    # Print summary report
    print("\n" + "="*60)
    print("📊 APM FOLDER ANALYSIS RESULTS")
    print("="*60)
    print(f"📁 Total Subdirectories: {len(folders)}")
    print(f"💾 Total Size: {total_size} MB ({total_size/1024:.2f} GB)")
    print(f"📄 Total Files: {total_files}")
    print(f"\n🔝 FOLDERS BY SIZE:")
    for i, (folder, size, count) in enumerate(sorted_data[:10], 1):
        percentage = (size / total_size * 100) if total_size > 0 else 0
        print(f"   {i:2d}. {folder:20s}: {size:4d} MB ({percentage:5.1f}%) - {count:4d} files")
    
    print(f"\n✅ Bar charts saved:")
    print(f"   📊 Size chart: {size_chart_path}")
    print(f"   📈 Count chart: {count_chart_path}")
    print("="*60)
    
    return size_chart_path, count_chart_path

if __name__ == "__main__":
    print("🎯 APM Folder Size Analyzer")
    print("="*40)
    analyze_apm_folder()