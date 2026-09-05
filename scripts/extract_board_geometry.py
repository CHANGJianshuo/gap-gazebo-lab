#!/usr/bin/env python3
"""Extract the supplied competition PDF at its native scale; never redraw it."""

import argparse
import hashlib
import json
from pathlib import Path

import fitz


ROOT = Path(__file__).resolve().parents[1]
MM_PER_PT = 25.4 / 72.0
LABELS = {
    1: {0: "base", 1: "initial_zone", 2: "A", 3: "B", 4: "C", 5: "D", 6: "T0"},
    2: {
        0: "base", 1: "initial_zone", 2: "A", 3: "B", 4: "C", 5: "D",
        8: "P1_outer", 9: "P1_inner", 10: "P2_outer", 11: "P2_inner",
        12: "P3_outer", 13: "P3_inner",
    },
}
def geometry_value(value):
    if isinstance(value, (fitz.Point, fitz.Rect)):
        return [float(x) for x in value]
    if isinstance(value, fitz.Quad):
        return [geometry_value(p) for p in (value.ul, value.ur, value.ll, value.lr)]
    if isinstance(value, (tuple, list)):
        return [geometry_value(x) for x in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pdf", type=Path, default=ROOT / "《2026年挑战赛任务底图》.pdf")
    parser.add_argument("--output", type=Path, default=ROOT / "docs/competition/extracted")
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--dpi", type=int, default=144)
    args = parser.parse_args()
    digest = hashlib.sha256(args.pdf.read_bytes()).hexdigest()
    if digest != args.expected_sha256:
        raise SystemExit("PDF checksum changed; review drawing labels before extracting a new revision.")
    args.output.mkdir(parents=True, exist_ok=True)
    result = {
        "source_filename": args.pdf.name,
        "source_sha256": digest,
        "coordinate_convention": "PDF top-left origin; u right, v down; lengths in mm",
        "proposed_board_frame": "Origin at printed base centre on table; +x to PDF top, +y to PDF left, +z up",
        "conversion": "x=(base_v-v)/1000; y=(base_u-u)/1000; z=0, units m",
        "status": "Measured source geometry; rule contradictions remain unresolved",
        "pages": [],
    }
    with fitz.open(args.pdf) as doc:
        if len(doc) != 2:
            raise SystemExit("Expected the reviewed two-page PDF.")
        for page_number, page in enumerate(doc, start=1):
            drawings = page.get_drawings()
            base_rect = drawings[0]["rect"]
            base_u = (base_rect.x0 + base_rect.x1) / 2 * MM_PER_PT
            base_v = (base_rect.y0 + base_rect.y1) / 2 * MM_PER_PT
            shapes = []
            for index, name in LABELS[page_number].items():
                drawing = drawings[index]
                rect = drawing["rect"]
                u = (rect.x0 + rect.x1) / 2 * MM_PER_PT
                v = (rect.y0 + rect.y1) / 2 * MM_PER_PT
                shapes.append({
                    "name": name,
                    "drawing_index": index,
                    "shape": "circle" if name == "base" or name.startswith("P") else "rounded_rectangle",
                    "bbox_mm": [round(x * MM_PER_PT, 6) for x in rect],
                    "size_uv_mm": [round(rect.width * MM_PER_PT, 6), round(rect.height * MM_PER_PT, 6)],
                    "center_uv_mm": [round(u, 6), round(v, 6)],
                    "center_board_xyz_m": [round((base_v - v) / 1000, 9), round((base_u - u) / 1000, 9), 0.0],
                    "stroke_width_mm": round(drawing["width"] * MM_PER_PT, 6),
                    "fill_rgb": geometry_value(drawing["fill"]),
                    "path_items_pdf_pt": geometry_value(drawing["items"]),
                })
            stem = "basic" if page_number == 1 else "sequence"
            png_path = args.output / f"board_{stem}.png"
            page.get_pixmap(dpi=args.dpi, alpha=False).save(png_path)
            result["pages"].append({
                "page": page_number,
                "task": stem,
                "size_uv_mm": [round(page.rect.width * MM_PER_PT, 6), round(page.rect.height * MM_PER_PT, 6)],
                "base_center_uv_mm": [base_u, base_v],
                "text": page.get_text(),
                "shapes": shapes,
                "render": {"path": png_path.name, "dpi": args.dpi, "sha256": hashlib.sha256(png_path.read_bytes()).hexdigest()},
            })
    path = args.output / "board_geometry.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(f"Extracted {len(result['pages'])} pages to {path}")


if __name__ == "__main__":
    main()
