#!/usr/bin/env python3
"""Headless smoke test for the Cleaning Queens site + admin suite."""
from playwright.sync_api import sync_playwright

BASE = "http://127.0.0.1:8090/preview/cleaning-queens/"
fails = []

def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        fails.append(name)

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page()
    errors = []
    pg.on("pageerror", lambda e: errors.append(str(e)))

    # public site
    pg.goto(BASE, wait_until="networkidle")
    check("banner has reviews", len(pg.inner_text("#bannerTrack")) > 100)
    check("services rendered", pg.locator("#svcGrid .card").count() == 5)
    check("real phone present", "(816) 269-5228" in pg.content())
    check("brand image loads", pg.evaluate("document.querySelector('.brandimg img').naturalWidth") > 100)
    check("work photo loads", pg.evaluate("document.querySelector('#work img').naturalWidth") > 100)
    check("reviews grid", pg.locator("#revGrid .rev").count() == 6)
    check("edit bar hidden logged out", not pg.locator("#editBar.show").count())
    # booking form
    pg.fill("#bf-name", "Test User")
    pg.fill("#bf-phone", "(555) 000-1111")
    pg.click("#bookForm button[type=submit]")
    check("booking confirmation", pg.locator("#bookDone").is_visible())

    # admin sign-in
    pg.goto(BASE + "admin.html", wait_until="networkidle")
    pg.fill("#lg-user", "admin")
    pg.fill("#lg-pass", "queen123")
    pg.click("#loginForm button")
    pg.wait_for_timeout(300)
    check("login shows app", pg.locator("#app").is_visible())
    check("dashboard stats", pg.locator(".stat").count() >= 5)
    check("booking became lead", "Test User" in pg.inner_text("#view"))

    # walk every view
    for v in ["sched", "jobs", "clients", "pay", "reviews", "staff", "settings"]:
        pg.click(f"#navi button[data-view={v}]")
        pg.wait_for_timeout(150)
        check(f"view {v} renders", len(pg.inner_text("#view").strip()) > 30)

    # record a payment
    pg.click("#navi button[data-view=pay]")
    pg.wait_for_timeout(150)
    before = pg.locator(".badge.paid").count()
    if pg.locator("[data-pay]").count():
        pg.locator("[data-pay]").first.click()
        pg.wait_for_timeout(200)
        check("payment recorded", pg.locator(".badge.paid").count() == before + 1)

    # admin edit mode on public site (same context = session persists)
    pg.goto(BASE, wait_until="networkidle")
    check("edit bar visible for admin", pg.locator("#editBar.show").count() == 1)
    pg.click("#editGo")
    pg.evaluate("document.querySelector('[data-edit=hero-h1]').innerHTML = 'EDITED HEADLINE TEST'")
    pg.click("#editSave")
    pg.reload(wait_until="networkidle")
    check("edit persisted after reload", "EDITED HEADLINE TEST" in pg.inner_text("h1"))

    check("no JS page errors", not errors)
    if errors:
        print("JS errors:", errors[:5])
    b.close()

print(("ALL OK" if not fails else f"{len(fails)} FAILURES"))
