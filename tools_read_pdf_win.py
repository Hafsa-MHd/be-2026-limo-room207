#!/usr/bin/env python3
from pathlib import Path

from pypdf import PdfReader

path = Path(r"c:\Users\ha054\Downloads\BE véhicule autonome") / "commande_polaire_robot (1).pdf"
print("exists", path.exists(), path)
reader = PdfReader(str(path))
print("pages", len(reader.pages))
out = Path(r"c:\Users\ha054\Downloads\BE véhicule autonome") / "commande_polaire_extract.txt"
chunks = []
for i, page in enumerate(reader.pages):
    text = page.extract_text() or ""
    chunks.append(f"\n===== PAGE {i+1} =====\n{text}")
out.write_text("\n".join(chunks), encoding="utf-8")
print(out)
print("\n".join(chunks))
