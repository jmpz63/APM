#!/usr/bin/env python3
"""Download full-size public photos from The Cleaning Queens KC Facebook page
into the cleaning-queens preview site's img/ folder."""
import os
import re
from playwright.sync_api import sync_playwright

OUT = "/home/jmpz63/forge_studio/previews/cleaning-queens/img"
os.makedirs(OUT, exist_ok=True)
UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36")

PAGES = [
    "https://www.facebook.com/p/The-Cleaning-Queens-KC-61583723273115/",
    "https://www.facebook.com/61583723273115/photos",
    "https://www.facebook.com/p/The-Cleaning-Queens-KC-61583723273115/photos/",
]

def best_urls(urls):
    """Keep fbcdn content images, prefer big variants, dedupe by photo id."""
    seen = {}
    for u in urls:
        if "scontent" not in u or "fbcdn.net" not in u:
            continue
        m = re.search(r"/(\d{9,})_", u)
        key = m.group(1) if m else u[:80]
        # prefer URLs without tiny thumbnail params
        score = 0
        if "s160x160" in u: score -= 2
        if "s960x960" in u or "s2048x2048" in u: score += 2
        if "p526x296" in u or "s720x720" in u: score += 1
        if key not in seen or score > seen[key][0]:
            seen[key] = (score, u)
    return [u for _, u in seen.values()]

with sync_playwright() as p:
    b = p.chromium.launch(args=["--disable-blink-features=AutomationControlled"])
    ctx = b.new_context(user_agent=UA, viewport={"width": 1280, "height": 3000}, locale="en-US")
    pg = ctx.new_page()
    collected = []
    for url in PAGES:
        try:
            pg.goto(url, wait_until="domcontentloaded", timeout=30000)
            pg.wait_for_timeout(4000)
            for _ in range(4):
                pg.mouse.wheel(0, 2500)
                pg.wait_for_timeout(1500)
            collected += pg.eval_on_selector_all(
                "img", "els => els.map(e => e.src).filter(s => s.startsWith('http'))")
        except Exception as e:
            print("!", url, "->", e)
    urls = best_urls(collected)
    print(f"{len(urls)} unique photos found")
    n = 0
    for u in urls:
        try:
            resp = ctx.request.get(u)
            if resp.status != 200:
                continue
            body = resp.body()
            if len(body) < 15000:   # skip tiny thumbnails/icons
                continue
            n += 1
            fn = os.path.join(OUT, f"fb_{n:02d}.jpg")
            with open(fn, "wb") as f:
                f.write(body)
            print(f"saved {fn} ({len(body)//1024} KB) <- {u[:90]}")
        except Exception as e:
            print("! dl fail:", e)
    b.close()
print("done")
