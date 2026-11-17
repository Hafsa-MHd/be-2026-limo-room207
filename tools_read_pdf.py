#!/usr/bin/env python3
from pathlib import Path
import sys

path = str(Path(__file__).resolve().parent / 'commande_polaire_robot (1).pdf')

# try pdftotext via subprocess first content already attempted
try:
    from pypdf import PdfReader
except ImportError:
    try:
        from PyPDF2 import PdfReader
    except ImportError:
        PdfReader = None

if PdfReader is None:
    import subprocess

    r = subprocess.run(
        ["pdftotext", "-layout", path, "-"],
        capture_output=True,
        text=True,
    )
    print(r.stdout or r.stderr or "EMPTY")
    sys.exit(r.returncode)

reader = PdfReader(path)
print("pages", len(reader.pages))
for i, page in enumerate(reader.pages):
    print(f"\n===== PAGE {i+1} =====\n")
    print(page.extract_text() or "")
