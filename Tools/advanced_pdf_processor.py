#!/usr/bin/env python3
"""
🤖 APM Advanced PDF Processing System
"""
import sys
import os
import json
import argparse
from pathlib import Path
from datetime import datetime
import subprocess
import shutil

# Add local packages to path
sys.path.insert(0, '/home/arm1/.local/lib/python3.10/site-packages')

try:
    import pdfplumber
    HAS_PDFPLUMBER = True
except ImportError:
    HAS_PDFPLUMBER = False

try:
    import fitz  # PyMuPDF
    HAS_PYMUPDF = True
except ImportError:
    HAS_PYMUPDF = False

def get_capabilities():
    """Check available PDF processing capabilities."""
    return {
        'pdfplumber': HAS_PDFPLUMBER,
        'pymupdf': HAS_PYMUPDF,
        'tesseract_ocr': shutil.which('tesseract') is not None,
        'poppler_utils': shutil.which('pdftotext') is not None,
        'ocrmypdf': shutil.which('ocrmypdf') is not None,
        'qpdf': shutil.which('qpdf') is not None
    }

def process_pdf_basic(pdf_path):
    """Basic PDF processing with available tools."""
    pdf_file = Path(pdf_path)
    output_dir = pdf_file.parent / f"{pdf_file.stem}_analysis"
    output_dir.mkdir(exist_ok=True)
    
    results = {
        'document': {
            'filename': pdf_file.name,
            'processed_at': datetime.now().isoformat(),
            'capabilities': get_capabilities()
        },
        'content': {
            'text': {},
            'metadata': {}
        }
    }
    
    print(f"🔍 Processing PDF: {pdf_file.name}")
    print(f"📁 Output directory: {output_dir}")
    
    # Test PDFPlumber
    if HAS_PDFPLUMBER:
        try:
            with pdfplumber.open(pdf_file) as pdf:
                text_pages = []
                for page_num, page in enumerate(pdf.pages, 1):
                    page_text = page.extract_text()
                    if page_text:
                        text_pages.append({
                            'page': page_num,
                            'text': page_text[:500] + "..." if len(page_text) > 500 else page_text
                        })
                
                results['content']['text']['pdfplumber'] = {
                    'pages': len(text_pages),
                    'sample_text': text_pages[0]['text'] if text_pages else "No text found"
                }
                print(f"  ✅ PDFPlumber: {len(text_pages)} pages processed")
        except Exception as e:
            print(f"  ❌ PDFPlumber error: {e}")
    
    # Test PyMuPDF
    if HAS_PYMUPDF:
        try:
            doc = fitz.open(pdf_file)
            page_count = len(doc)
            first_page_text = doc[0].get_text()[:500] + "..." if len(doc) > 0 else "No content"
            
            results['content']['text']['pymupdf'] = {
                'pages': page_count,
                'sample_text': first_page_text
            }
            print(f"  ✅ PyMuPDF: {page_count} pages processed")
            doc.close()
        except Exception as e:
            print(f"  ❌ PyMuPDF error: {e}")
    
    # Test Poppler
    try:
        result = subprocess.run(
            ['pdftotext', '-l', '1', str(pdf_file), '-'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            sample_text = result.stdout[:500] + "..." if len(result.stdout) > 500 else result.stdout
            results['content']['text']['poppler'] = {
                'sample_text': sample_text,
                'status': 'success'
            }
            print(f"  ✅ Poppler: Text extraction successful")
    except Exception as e:
        print(f"  ❌ Poppler error: {e}")
    
    # Save results
    output_file = output_dir / f"{pdf_file.stem}_analysis.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"  ✅ Results saved to: {output_file}")
    return results

def batch_process_pdfs(search_path, pattern="**/*.pdf"):
    """Process all PDFs matching pattern in search_path."""
    search_dir = Path(search_path)
    pdf_files = sorted(search_dir.glob(pattern))
    
    print(f"🔍 Searching: {search_path}")
    print(f"📄 Found {len(pdf_files)} PDF files")
    print("=" * 60)
    
    results_summary = []
    for pdf_file in pdf_files:
        try:
            print(f"\n📄 Processing: {pdf_file.relative_to(search_dir)}")
            result = process_pdf_basic(str(pdf_file))
            results_summary.append({
                'file': str(pdf_file.relative_to(search_dir)),
                'status': 'success',
                'pages': result.get('content', {}).get('text', {}).get('pymupdf', {}).get('pages', 0)
            })
        except Exception as e:
            print(f"  ❌ Error: {e}")
            results_summary.append({
                'file': str(pdf_file.relative_to(search_dir)),
                'status': 'error',
                'error': str(e)
            })
    
    print("\n" + "=" * 60)
    print("📊 BATCH PROCESSING SUMMARY")
    print("=" * 60)
    success_count = sum(1 for r in results_summary if r['status'] == 'success')
    print(f"✅ Processed: {success_count}/{len(results_summary)} files")
    
    return results_summary

def main():
    parser = argparse.ArgumentParser(description="APM PDF Processor")
    parser.add_argument('pdf_file', nargs='?', help='PDF file to process')
    parser.add_argument('--test', action='store_true', help='Test capabilities')
    parser.add_argument('--batch', metavar='DIR', help='Process all PDFs in directory')
    parser.add_argument('--joints', action='store_true', help='Process all joint datasheets')
    parser.add_argument('--pattern', default='**/*.pdf', help='File pattern for batch processing')
    
    args = parser.parse_args()
    
    if args.test or (not args.pdf_file and not args.batch and not args.joints):
        print("🤖 APM Advanced PDF Processor")
        print("=" * 50)
        capabilities = get_capabilities()
        print("Available capabilities:")
        for tool, available in capabilities.items():
            status = "✅" if available else "❌"
            print(f"  {status} {tool}")
        
        if not args.pdf_file and not args.batch and not args.joints:
            print("\n📖 Usage:")
            print("  Single file: python3 advanced_pdf_processor.py <file.pdf>")
            print("  Batch:       python3 advanced_pdf_processor.py --batch <dir>")
            print("  All joints:  python3 advanced_pdf_processor.py --joints")
            return
    
    if args.joints:
        # Process all joint datasheets
        joints_dir = Path('/home/arm1/APM/Projects/Active/wall_panel_manufacturing/Onboarding/04_Hardware_Firmware_Network_Architecture/joint_datasheets')
        batch_process_pdfs(joints_dir)
        return
    
    if args.batch:
        batch_process_pdfs(args.batch, args.pattern)
        return
    
    pdf_path = Path(args.pdf_file)
    if not pdf_path.exists():
        print(f"❌ Error: File not found: {pdf_path}")
        return
    
    process_pdf_basic(str(pdf_path))

if __name__ == "__main__":
    main()
