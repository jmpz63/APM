#!/usr/bin/env python3
"""Validate mermaid code blocks in a markdown file render without parse errors."""
import re
import sys
from playwright.sync_api import sync_playwright

md = open(sys.argv[1], encoding="utf-8").read()
blocks = re.findall(r"```mermaid\n(.*?)```", md, re.S)
if not blocks:
    print("no mermaid blocks found")
    sys.exit(1)

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page()
    pg.goto("about:blank")
    pg.add_script_tag(url="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js")
    pg.evaluate("mermaid.initialize({startOnLoad:false})")
    ok = True
    for i, code in enumerate(blocks):
        try:
            pg.evaluate("async c => { await mermaid.parse(c); }", code)
            svg = pg.evaluate("async c => (await mermaid.render('m%d', c)).svg.length" % i, code)
            print(f"block {i}: PARSE OK, svg {svg} chars")
        except Exception as e:
            ok = False
            print(f"block {i}: FAIL -> {str(e)[:400]}")
    b.close()
sys.exit(0 if ok else 1)
