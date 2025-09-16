from __future__ import annotations

import pytest

playwright = pytest.importorskip(
    "playwright.sync_api", reason="Playwright not available"
)


@pytest.mark.skip(
    reason="Playwright smoke test requires running the frontend dev server"
)
def test_frontend_workflow():
    from playwright.sync_api import sync_playwright  # type: ignore

    with sync_playwright() as p:  # pragma: no cover - manual smoke path
        browser = p.chromium.launch()
        page = browser.new_page()
        page.goto("http://localhost:5173/spaces", wait_until="networkidle")
        browser.close()
