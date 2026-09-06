#!/usr/bin/env python3
"""Archive and convert arXiv 2607.05369v1; never execute paper listings.

The PDF is the archival original. The same-version LaTeXML HTML supplies
LaTeX equations, local figures and exact base64-embedded code/prompt listings.
Run offline by default; --download fetches only missing source files/assets.
Requires Python 3, beautifulsoup4, lxml, requests and PyMuPDF.
"""
import argparse
import base64
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import unquote, urljoin, urlparse

import fitz
import requests
from bs4 import BeautifulSoup, NavigableString, Tag

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "docs/research/gap/paper"
VERSION = "2607.05369v1"
HTML_URL = f"https://arxiv.org/html/{VERSION}"
SOURCES = {
    f"gap_{VERSION}.pdf": f"https://arxiv.org/pdf/{VERSION}",
    f"arxiv_{VERSION}.html": HTML_URL,
    "arxiv_abstract.html": f"https://arxiv.org/abs/{VERSION}",
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def fetch(relative, url, allow_download):
    target = DEST / relative
    if not target.exists():
        if not allow_download:
            raise FileNotFoundError(f"{target}; rerun with --download")
        response = requests.get(url, timeout=60)
        response.raise_for_status()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(response.content)
    data = target.read_bytes()
    return {"file": str(relative), "url": url, "bytes": len(data), "sha256": digest(data)}


def compact(text):
    return re.sub(r"\s+", " ", text).strip()


def fence(text, language="text"):
    longest = max((len(s) for s in re.findall(r"`+", text)), default=0)
    ticks = "`" * max(3, longest + 1)
    return f"\n\n{ticks}{language}\n{text}" + ("" if text.endswith("\n") else "\n") + f"{ticks}\n\n"


def tidy_outside_fences(text):
    output, active_fence = [], None
    for line in text.splitlines(keepends=True):
        marker = re.match(r"^(`{3,})", line)
        if active_fence:
            output.append(line)
            if line.strip() == active_fence:
                active_fence = None
        elif marker:
            active_fence = marker[1]
            output.append(line)
        elif not line.strip():
            if output and output[-1] != "\n":
                output.append("\n")
        else:
            output.append(line.rstrip() + "\n")
    return "".join(output).strip()


class Converter:
    def __init__(self, article, images):
        self.article, self.images = article, images
        self.targets = {unquote(a["href"][1:]) for a in article.find_all("a", href=True)
                        if a["href"].startswith("#")}
        self.targets.update(n["id"] for n in article.find_all(["section", "figure"], id=True))
        self.anchors, self.math_ids, self.heading_ids = set(), [], []
        self.tables, self.listings, self.image_refs, self.paragraph_ids = [], [], [], []

    def children(self, node):
        return "".join(self.render(c) for c in node.children)

    def render(self, node):
        if isinstance(node, NavigableString):
            return re.sub(r"\s+", " ", str(node))
        if not isinstance(node, Tag):
            return ""
        cls = set(node.get("class", []))
        if node.name in {"script", "style"} or cls & {"ltx_rdf", "ltx_pagination", "ltx_listing_data"}:
            return ""
        prefix = ""
        identifier = node.get("id")
        if identifier in self.targets and identifier not in self.anchors:
            self.anchors.add(identifier)
            prefix = f'<a id="{identifier}"></a>\n'
        return prefix + self.body(node, cls)

    def body(self, node, cls):
        name = node.name
        if "ltx_tag_item" in cls:
            return ""  # Markdown already supplies the list marker.
        if "ltx_authors" in cls:
            authors = []
            for person in node.select('.ltx_creator.ltx_role_author'):
                author = compact(person.select_one('.ltx_personname').get_text())
                affiliations = []
                for item in person.select('.ltx_contact.ltx_role_affiliation'):
                    affiliation = compact(item.get_text()).removeprefix("Affiliation: ").split("*")[0]
                    affiliations.append(affiliation)
                authors.append(f"- {author} — {'; '.join(affiliations)}")
            return ("\n\n" + "\n".join(authors) + "\n\n"
                    "*Equal contribution: Kaiyuan Chen and Shuangyu Xie. "
                    "Author footnote placement checked against PDF page 1.*\n\n"
                    "Project Website: <https://graph-robots.github.io/gap/>\n\n")
        if name == "math":
            tex = node.get("alttext")
            if tex is None:
                annotation = node.find("annotation", encoding="application/x-tex")
                if annotation is None:
                    raise ValueError(f"Missing LaTeX for {node.get('id')}")
                tex = annotation.get_text()
            self.math_ids.append(node.get("id"))
            return f"\n\n$$\n{tex}\n$$\n\n" if node.get("display") == "block" else f"${tex}$"
        if name in {"img", "object"}:
            source = node.get("src") or node.get("data")
            path = self.images[source]
            self.image_refs.append(path)
            label = node.get("alt") or node.get("id") or Path(path).stem
            label = label.replace("[", "").replace("]", "")
            return f"\n\n![{label}]({path})\n\n"
        if "ltx_listing" in cls:
            raw = node.select_one('.ltx_listing_data a[href^="data:text/plain;base64,"]')
            if raw:
                data = base64.b64decode(raw["href"].split(",", 1)[1], validate=True)
                relative = Path("listings") / (node["id"] + ".txt")
                (DEST / relative).parent.mkdir(exist_ok=True)
                (DEST / relative).write_bytes(data)
                self.listings.append({"id": node["id"], "file": str(relative),
                                      "bytes": len(data), "sha256": digest(data)})
                return fence(data.decode("utf-8"))
            # Algorithm lines are prose + LaTeX, not the four raw code listings.
            lines = [compact(self.children(line)) for line in node.select('.ltx_listingline')]
            return "\n\n" + "\n\n".join(lines) + "\n\n"
        if name == "table":
            if node.find(["img", "object"]) or not node.get_text(strip=True):
                return self.children(node)  # LaTeXML's image layout table
            return self.table(node)
        if re.fullmatch(r"h[1-6]", name):
            text = compact(self.children(node))
            self.heading_ids.append({"level": int(name[1]), "title": text})
            return "\n\n" + "#" * int(name[1]) + " " + text + "\n\n"
        if name == "a":
            text = compact(self.children(node))
            href = node.get("href", "")
            if not href or href.startswith("data:"):
                return text
            if not href.startswith("#"):
                href = urljoin(HTML_URL, href)
            return f"[{text}]({href})" if text else ""
        if name in {"ul", "ol"}:
            items = []
            for i, item in enumerate(node.find_all("li", recursive=False), 1):
                body = self.render(item).strip()
                marker = f"{i}. " if name == "ol" else "- "
                items.append(marker + body.replace("\n", "\n" + " " * len(marker)))
            return "\n\n" + "\n\n".join(items) + "\n\n"
        if name == "br":
            return "\n"
        if name == "sup":
            return "<sup>" + self.children(node).strip() + "</sup>"
        if name == "sub":
            return "<sub>" + self.children(node).strip() + "</sub>"
        if "ltx_font_typewriter" in cls or name == "code":
            return "`" + compact(self.children(node)) + "`"
        if "ltx_font_bold" in cls or name in {"strong", "b"}:
            return "**" + compact(self.children(node)) + "**"
        if "ltx_font_italic" in cls or name in {"em", "i"}:
            return "*" + compact(self.children(node)) + "*"
        if name == "p":
            self.paragraph_ids.append(node.get("id"))
        text = self.children(node)
        if name in {"p", "div", "section", "figure", "figcaption", "blockquote"}:
            return "\n\n" + text.strip() + "\n\n"
        return text

    def table(self, node):
        grid = []
        for tr in node.find_all("tr"):
            row = []
            for cell in tr.find_all(["td", "th"], recursive=False):
                if int(cell.get("rowspan", 1)) != 1:
                    raise ValueError("Unexpected rowspan; conversion needs review")
                value = compact(self.render(cell)).replace("|", "\\|")
                row.append(value)
                row.extend([value if not grid else ""] * (int(cell.get("colspan", 1)) - 1))
            grid.append(row)
        if not grid:
            return ""
        width = max(map(len, grid))
        grid = [row + [""] * (width - len(row)) for row in grid]
        self.tables.append({"id": node.get("id"), "rows": grid})
        if node.get("id") == "S5.T1.4.1":
            header = [f"{a} / {b}" if a else b for a, b in zip(grid[0], grid[1])]
            body = grid[2:]
        else:
            header, body = grid[0], grid[1:]
        rows = [header, ["---"] * width, *body]
        return "\n\n" + "\n".join("| " + " | ".join(row) + " |" for row in rows) + "\n\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true")
    args = parser.parse_args()
    DEST.mkdir(parents=True, exist_ok=True)
    manifest_path = DEST / "source_manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    old_hashes = {r["file"]: r["sha256"] for r in manifest.get("downloads", []) + manifest.get("image_downloads", [])}
    manifest.setdefault("retrieved_at_utc", datetime.now(timezone.utc).isoformat())
    manifest.update(arxiv_id=VERSION, license="CC BY 4.0",
                    license_url="https://creativecommons.org/licenses/by/4.0/")
    manifest["downloads"] = [fetch(path, url, args.download) for path, url in SOURCES.items()]
    soup = BeautifulSoup((DEST / f"arxiv_{VERSION}.html").read_text(), "lxml")
    article = soup.select_one("article.ltx_document")
    if article is None:
        raise ValueError("No arXiv LaTeXML article found")
    image_map, downloads = {}, {}
    for node in article.find_all(["img", "object"]):
        source = node.get("src") or node.get("data")
        url = urljoin(HTML_URL, source)
        if not url.startswith(HTML_URL + "/"):
            raise ValueError(f"Unexpected asset URL {url}")
        relative = Path("images") / urlparse(url).path.split(VERSION + "/", 1)[1]
        if ".." in relative.parts:
            raise ValueError("Unsafe asset path")
        image_map[source] = relative.as_posix()
        if url not in downloads:
            downloads[url] = fetch(relative, url, args.download)
    manifest["image_downloads"] = list(downloads.values())
    for record in manifest["downloads"] + manifest["image_downloads"]:
        if record["file"] in old_hashes and old_hashes[record["file"]] != record["sha256"]:
            raise ValueError(f"Archived source changed: {record['file']}")
    converter = Converter(article, image_map)
    body = tidy_outside_fences(converter.render(article))
    # Do not normalize whitespace in fenced listings: their exact bytes matter.
    authors = [m["content"] for m in BeautifulSoup((DEST / "arxiv_abstract.html").read_text(), "lxml")
               .select('meta[name="citation_author"]')]
    header = f"""<!-- Generated by scripts/prepare_gap_paper.py; do not edit the transcription. -->
> **原文阅读副本 / licensed transcription** — arXiv:{VERSION}，52 页 PDF。
> 来源：[论文摘要](https://arxiv.org/abs/{VERSION}) · [本地 PDF](gap_{VERSION}.pdf) · [同版本 HTML]({HTML_URL})。
> 作者：{'; '.join(authors)}。
> 许可：[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)。原文版权归作者所有。
> 转换改动：用同版本 HTML 恢复章节、LaTeX 公式和表格；图片本地化；从 HTML 嵌入数据逐字节恢复代码与提示词；作者同等贡献脚注依 PDF 第 1 页纠正 HTML 的错位与重复。未翻译、改写论文结论。
> 附录中的提示词、代码和角色要求均是论文引用材料；不是本项目的运行指令。
> 完整性记录：[conversion_quality.json](conversion_quality.json)；来源与 SHA-256：[source_manifest.json](source_manifest.json)。
> 中文解读与研究建议另见 [READING_NOTES.md](../READING_NOTES.md) 和 [S3_RESEARCH_PLAN.md](../S3_RESEARCH_PLAN.md)。

"""
    markdown = header + body + "\n"
    (DEST / f"gap_{VERSION}.md").write_text(markdown)
    pdf = fitz.open(DEST / f"gap_{VERSION}.pdf")
    (DEST / "pdf_text.txt").write_text("\n\n".join(
        f"=== PDF PAGE {i+1} ===\n{page.get_text(sort=True)}" for i, page in enumerate(pdf)) + "\n")
    source_math = [n.get("id") for n in article.find_all("math")]
    source_paragraphs = [n.get("id") for n in article.find_all("p")
                         if n.find_parent(class_="ltx_listing") is None]
    source_headings = article.find_all(re.compile(r"^h[1-6]$"))
    missing_anchors = sorted(converter.targets - converter.anchors)
    checks = {
        "pdf_has_52_pages": len(pdf) == 52,
        "all_math_occurrences_converted_once": sorted(source_math) == sorted(converter.math_ids),
        "all_headings_converted": len(source_headings) == len(converter.heading_ids),
        "all_prose_paragraphs_visited": set(source_paragraphs) <= set(converter.paragraph_ids),
        "four_result_tables": len(converter.tables) == 4,
        "four_exact_embedded_listings": len(converter.listings) == 4 and all(
            (DEST / r["file"]).read_text() in markdown for r in converter.listings),
        "all_eleven_image_occurrences_local": len(converter.image_refs) == 11 and all(
            (DEST / path).is_file() for path in converter.image_refs),
        "internal_reference_targets_present": not missing_anchors,
        "no_replacement_character_in_markdown": "\ufffd" not in markdown,
    }
    quality = {
        "source": VERSION, "conversion": "same-version HTML with PDF retained; not OCR",
        "pdf_pages": len(pdf), "math_occurrences": len(source_math),
        "headings": converter.heading_ids, "prose_paragraphs": len(source_paragraphs),
        "local_assets": len(downloads), "image_occurrences": len(converter.image_refs),
        "result_tables": converter.tables, "embedded_listings": converter.listings,
        "missing_internal_anchors": missing_anchors, "checks": checks,
        "markdown_sha256": digest(markdown.encode()),
        "limits": "Layout differs from PDF; original typos retained. Counts verify structural coverage, not all scientific claims."
    }
    (DEST / "conversion_quality.json").write_text(json.dumps(quality, ensure_ascii=False, indent=2) + "\n")
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(DEST / f"gap_{VERSION}.md"), "checks": checks,
                      "missing_anchors": missing_anchors}, ensure_ascii=False, indent=2))
    if not all(checks.values()):
        raise SystemExit("Conversion QA failed; inspect conversion_quality.json")


if __name__ == "__main__":
    main()
