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

def main():
    parser = argparse.ArgumentParser(description="APM PDF Processor")
    parser.add_argument('pdf_file', nargs='?', help='PDF file to process')
    parser.add_argument('--test', action='store_true', help='Test capabilities')
    
    args = parser.parse_args()
    
    if args.test or not args.pdf_file:
        print("🤖 APM Advanced PDF Processor")
        print("=" * 50)
        capabilities = get_capabilities()
        print("Available capabilities:")
        for tool, available in capabilities.items():
            status = "✅" if available else "❌"
            print(f"  {status} {tool}")
        
        if not args.pdf_file:
            return
    
    pdf_path = Path(args.pdf_file)
    if not pdf_path.exists():
        print(f"❌ Error: File not found: {pdf_path}")
        return
    
    process_pdf_basic(str(pdf_path))

if __name__ == "__main__":
    main()
