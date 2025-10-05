#!/usr/bin/env python3
"""APM PDF Integration - Process PDFs and integrate into APM Knowledge Base"""
import subprocess
import sys
from pathlib import Path

def main():
    apm_tools = Path('/home/arm1/APM/Tools')
    pdf_processor = apm_tools / 'advanced_pdf_processor.py'
    
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        print("🤖 APM PDF Integration System")
        print("=" * 50)
        subprocess.run(['python3', str(pdf_processor), '--test'])
        print("\n📄 PDF Integration Ready!")
        print("Usage: python3 apm_pdf_integration.py <pdf_file>")
    elif len(sys.argv) > 1:
        pdf_file = sys.argv[1]
        print(f"📄 Processing PDF: {pdf_file}")
        subprocess.run(['python3', str(pdf_processor), pdf_file])
    else:
        print("Usage: python3 apm_pdf_integration.py <pdf_file>")
        print("       python3 apm_pdf_integration.py test")

if __name__ == "__main__":
    main()
