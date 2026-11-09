import os
import sys

import fitz


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: render_pdf_to_png.py input.pdf output_dir")

    pdf_path = sys.argv[1]
    out_dir = sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    doc = fitz.open(pdf_path)
    for index, page in enumerate(doc):
        pix = page.get_pixmap(matrix=fitz.Matrix(1.5, 1.5), alpha=False)
        out_path = os.path.join(out_dir, f"page-{index + 1}.png")
        pix.save(out_path)
        print(out_path)


if __name__ == "__main__":
    main()
