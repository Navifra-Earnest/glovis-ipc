#!/usr/bin/env python3
"""design.src.html + docs/dot/*.dot → docs/navi_design.pdf

개발 PC 에서 돌린다. 보드 빌드(make all)와는 무관하다 — 보드에는 graphviz 도
weasyprint 도 없다.

    python3 tools/make_pdf.py

의존: graphviz(dot), weasyprint, Noto Sans CJK KR 폰트

  sudo apt install graphviz fonts-noto-cjk
  pip install --user weasyprint

동작:
  1) docs/dot/*.dot 을 SVG 로 렌더
  2) 본문의 __SVG:이름__ 을 해당 SVG 로 치환 (인라인 — 외부 파일 참조 없음)
     ⚠ Graphviz 가 붙이는 width/height 는 지운다. 안 지우면 SVG 가 제 크기를
       고집해서 A4 폭을 넘고 오른쪽이 잘린다. viewBox 만 남기고 CSS 로 맞춘다.
  3) __DATE__ / __COMMIT__ 치환
  4) WeasyPrint 로 PDF
"""
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "docs" / "design.src.html"
DOT_DIR = ROOT / "docs" / "dot"
OUT = ROOT / "docs" / "navi_design.pdf"


def need(cmd):
    if subprocess.run(["which", cmd], capture_output=True).returncode:
        sys.exit(f"🔴 {cmd} 가 없다. graphviz / weasyprint 설치가 필요하다.")


def render_svg(dot_file):
    """dot → SVG 문자열. XML 선언과 DOCTYPE 은 인라인에 못 쓰니 떼어낸다."""
    r = subprocess.run(["dot", "-Tsvg", str(dot_file)], capture_output=True, text=True)
    if r.returncode:
        sys.exit(f"🔴 {dot_file.name} 렌더 실패:\n{r.stderr}")
    svg = r.stdout
    svg = re.sub(r"<\?xml[^>]*\?>", "", svg)
    svg = re.sub(r"<!DOCTYPE[^>]*>", "", svg, flags=re.I)
    svg = re.sub(r"<!--.*?-->", "", svg, flags=re.S)
    # 고정 크기를 지운다 — viewBox 와 CSS(max-width:100%)로 페이지에 맞춘다
    svg = re.sub(r'(<svg\b[^>]*?)\swidth="[^"]*"', r"\1", svg, count=1)
    svg = re.sub(r'(<svg\b[^>]*?)\sheight="[^"]*"', r"\1", svg, count=1)
    return svg.strip()


def git(*args, default=""):
    r = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else default


def main():
    need("dot")
    need("weasyprint")
    if not SRC.exists():
        sys.exit(f"🔴 본문이 없다: {SRC}")

    html = SRC.read_text(encoding="utf-8")

    for dot_file in sorted(DOT_DIR.glob("*.dot")):
        marker = f"__SVG:{dot_file.stem}__"
        if marker not in html:
            print(f"[!] {dot_file.name} 은 본문에서 안 쓴다 — 건너뛴다")
            continue
        html = html.replace(marker, render_svg(dot_file))
        print(f"  그림  {dot_file.stem}")

    left = re.findall(r"__SVG:(\w+)__", html)
    if left:
        sys.exit(f"🔴 본문이 참조하는 그림이 없다: {', '.join(left)}")

    commit = git("rev-parse", "--short", "HEAD", default="(git 없음)")
    dirty = git("status", "--porcelain")
    if dirty:
        commit += " + 커밋 안 된 변경"
    html = html.replace("__DATE__", datetime.now().strftime("%Y-%m-%d"))
    html = html.replace("__COMMIT__", commit)

    tmp = ROOT / "docs" / ".design.build.html"
    tmp.write_text(html, encoding="utf-8")
    r = subprocess.run(["weasyprint", str(tmp), str(OUT)], capture_output=True, text=True)
    tmp.unlink(missing_ok=True)
    if r.returncode:
        sys.exit(f"🔴 PDF 생성 실패:\n{r.stderr}")
    if r.stderr.strip():
        print(r.stderr.strip())

    print(f"\n완료: {OUT.relative_to(ROOT)}  ({OUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
