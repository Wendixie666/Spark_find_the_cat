from pathlib import Path

import fitz


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "docs" / "project.pdf"
OUTPUT = ROOT / "output" / "pdf" / "project_sanitized.pdf"


def rect(x0: float, y0: float, x1: float, y1: float) -> fitz.Rect:
    return fitz.Rect(x0, y0, x1, y1)


doc = fitz.open(SOURCE)

# Cover page: remove the parenthesized ID labels and numbers, preserving names.
cover_redactions = [
    rect(194.0, 153.2, 280.0, 167.5),
    rect(371.5, 153.2, 459.0, 167.5),
    rect(264.5, 167.2, 352.0, 181.5),
]

# Team contribution table: remove the Student ID heading and all three IDs.
table_redactions = [
    rect(161.3, 282.0, 223.5, 291.7),
    rect(161.3, 295.8, 211.1, 305.8),
    rect(161.3, 309.7, 211.1, 319.8),
    rect(161.3, 323.7, 211.1, 333.8),
]

for r in cover_redactions:
    doc[0].add_redact_annot(r, fill=(1, 1, 1))
for r in table_redactions:
    doc[9].add_redact_annot(r, fill=(1, 1, 1))

for page in (doc[0], doc[9]):
    page.apply_redactions(
        images=fitz.PDF_REDACT_IMAGE_NONE,
        graphics=fitz.PDF_REDACT_LINE_ART_NONE,
        text=fitz.PDF_REDACT_TEXT_REMOVE,
    )

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
doc.save(OUTPUT, garbage=4, deflate=True)
doc.close()
print(OUTPUT)
