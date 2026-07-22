#!/usr/bin/env python3
"""Attempt to scrape public info from The Cleaning Queens KC Facebook page."""
import json
from playwright.sync_api import sync_playwright

URLS = [
    "https://www.facebook.com/p/The-Cleaning-Queens-KC-61583723273115/",
    "https://m.facebook.com/p/The-Cleaning-Queens-KC-61583723273115/",
    "https://www.facebook.com/61583723273115",
]
UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36")

with sync_playwright() as p:
    b = p.chromium.launch(args=["--disable-blink-features=AutomationControlled"])
    ctx = b.new_context(user_agent=UA, viewport={"width": 1280, "height": 2000},
                        locale="en-US")
    pg = ctx.new_page()
    for url in URLS:
        try:
            pg.goto(url, wait_until="domcontentloaded", timeout=30000)
            pg.wait_for_timeout(4000)
            # dismiss login dialog if present
            for sel in ("div[aria-label='Close']", "[aria-label='Close']"):
                try:
                    if pg.locator(sel).count():
                        pg.locator(sel).first.click(timeout=2000)
                        pg.wait_for_timeout(1000)
                except Exception:
                    pass
            title = pg.title()
            text = pg.inner_text("body")[:6000]
            imgs = pg.eval_on_selector_all(
                "img", "els => els.map(e => e.src).filter(s => s.startsWith('http'))")[:40]
            print("=" * 70)
            print("URL:", url)
            print("TITLE:", title)
            print("IMAGES:", json.dumps(imgs, indent=0)[:3000])
            print("TEXT:")
            print(text)
        except Exception as e:
            print("!", url, "->", e)
    pg.screenshot(path="/tmp/cq_fb.png", full_page=False)
    b.close()
