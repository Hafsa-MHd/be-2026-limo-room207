#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq poppler-utils python3-pypdf2
which pdftotext
python3 -c 'from PyPDF2 import PdfReader; print("pypdf2 ok")'
pdftotext -layout "$ROOT/commande_polaire_robot (1).pdf" /tmp/polaire.txt
wc -l /tmp/polaire.txt
cat /tmp/polaire.txt
