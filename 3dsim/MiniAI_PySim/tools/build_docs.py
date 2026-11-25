# -*- coding: utf-8 -*-
"""Build the one-page quickstart PDF from the repository root README.md (up to the <!-- pdf-end --> marker).

    python 3dsim/MiniAI_PySim/tools/build_docs.py [--out <folder>]

Needs pandoc and a Chromium-based browser (Chrome / Edge / Chromium) on PATH, or set CHROME=<path>.
The PDF is a build product: it is not committed; GitHub Actions attaches it to each Release.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.normpath(os.path.join(HERE, "..", "..", "docs"))
README = os.path.normpath(os.path.join(HERE, "..", "..", "..", "README.md"))

CSS = """
body{font-family:"Noto Sans CJK JP","Noto Sans JP","Yu Gothic UI","Meiryo",sans-serif;font-size:9.6pt;line-height:1.42;color:#222;margin:0}
h1{font-size:15pt;margin:0 0 4px;color:#0b4f8a}
h2{font-size:11pt;margin:9px 0 3px;padding:2px 6px;background:#e8f0f8;border-left:4px solid #0b4f8a}
pre{background:#f3f3f3;border:1px solid #ddd;padding:4px 6px;font-size:8.6pt;white-space:pre-wrap;margin:3px 0}
code{font-family:Consolas,"Noto Sans Mono CJK JP",monospace;font-size:8.8pt}
p,ul,ol{margin:3px 0} ul,ol{padding-left:20px} li{margin:1px 0}
@page{size:A4;margin:12mm 14mm}
"""


def find_chrome():
    env = os.environ.get("CHROME")
    if env:
        return env
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "chrome", "msedge"):
        p = shutil.which(name)
        if p:
            return p
    for p in (r"C:\Program Files\Google\Chrome\Application\chrome.exe",
              r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
              r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"):
        if os.path.exists(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(DOCS, "_build"))
    a = ap.parse_args()
    if not shutil.which("pandoc"):
        sys.exit("pandoc が見つかりません（https://pandoc.org/）")
    chrome = find_chrome()
    if not chrome:
        sys.exit("Chrome / Edge / Chromium が見つかりません（環境変数 CHROME でパスを指定できます）")
    os.makedirs(a.out, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        css = os.path.join(tmp, "s.css")
        with open(css, "w", encoding="utf-8") as fh:
            fh.write(CSS)
        with open(README, encoding="utf-8") as fh:
            text = fh.read().split("<!-- pdf-end")[0]
        src = os.path.join(tmp, "quickstart.md")
        with open(src, "w", encoding="utf-8") as fh:
            fh.write(text)
        html = os.path.join(tmp, "quickstart.html")
        subprocess.run(["pandoc", src, "-s", "--embed-resources", "--css", css,
                        "--metadata", "pagetitle=quickstart", "-o", html], check=True)
        pdf = os.path.abspath(os.path.join(a.out, "quickstart.pdf"))
        subprocess.run([chrome, "--headless", "--disable-gpu", "--no-sandbox", "--no-pdf-header-footer",
                        "--print-to-pdf=" + pdf, "file://" + html.replace("\\", "/")], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("PDF ->", pdf)


if __name__ == "__main__":
    main()
