import math
import os
import sys

from PIL import Image, ImageDraw


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: make_contact_sheet.py png_dir")

    png_dir = sys.argv[1]
    files = sorted(
        [
            os.path.join(png_dir, name)
            for name in os.listdir(png_dir)
            if name.startswith("page-") and name.endswith(".png")
        ],
        key=lambda path: int(os.path.basename(path)[5:-4]),
    )
    images = [Image.open(path).convert("RGB") for path in files]
    thumbs = []
    thumb_width = 330
    pad = 24
    label_height = 28

    for index, image in enumerate(images, 1):
        thumb = image.copy()
        thumb.thumbnail((thumb_width, 460))
        canvas = Image.new("RGB", (thumb_width, thumb.height + label_height), "white")
        canvas.paste(thumb, ((thumb_width - thumb.width) // 2, label_height))
        ImageDraw.Draw(canvas).text((8, 6), f"Page {index}", fill=(0, 0, 0))
        thumbs.append(canvas)

    cols = 2
    rows = math.ceil(len(thumbs) / cols)
    cell_height = max(thumb.height for thumb in thumbs)
    sheet = Image.new(
        "RGB",
        (cols * thumb_width + (cols + 1) * pad, rows * cell_height + (rows + 1) * pad),
        "white",
    )

    for index, thumb in enumerate(thumbs):
        x = pad + (index % cols) * (thumb_width + pad)
        y = pad + (index // cols) * (cell_height + pad)
        sheet.paste(thumb, (x, y))

    out_path = os.path.join(png_dir, "contact-sheet.png")
    sheet.save(out_path)
    print(out_path)


if __name__ == "__main__":
    main()
