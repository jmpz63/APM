#!/usr/bin/env python3
"""Check mermaid_sandbox.html boots and renders a given example diagram."""
import sys
from playwright.sync_api import sync_playwright

example = sys.argv[1] if len(sys.argv) > 1 else "website_build"
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page()
    errs = []
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.goto(f"file:///tmp/mermaid_sandbox.html?example={example}")
    pg.wait_for_timeout(3000)
    svg = pg.evaluate('document.querySelector("#diagram svg") ? document.querySelector("#diagram svg").outerHTML.length : 0')
    status = pg.inner_text("#status")
    print("svg chars:", svg, "| status:", status, "| js errors:", errs[:2])
    print("BOOT OK" if svg > 10000 and not errs else "BOOT FAIL")
    b.close()
