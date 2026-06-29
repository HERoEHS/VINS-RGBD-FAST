#!/usr/bin/env python3
# Generate four A4-printable 6x6 AprilTag36h11 boards with unique ID ranges for GT.
#
# Output IDs:
#   AR1:   0..35
#   AR2:  36..71
#   AR3:  72..107
#   AR4: 108..143
#
# Row convention matches gt_apriltag_4board.py:
#   board row 0 is the physical bottom row, so the saved image displays row 5 at top.

import argparse
import os
from collections import Counter

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


BOARDS = [
    ("AR1", 0),
    ("AR2", 36),
    ("AR3", 72),
    ("AR4", 108),
]


def make_marker(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    marker = np.zeros((side_px, side_px), dtype=np.uint8)
    cv2.aruco.drawMarker(dictionary, tag_id, side_px, marker, 1)
    return marker


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    return dictionary, cv2.aruco.ArucoDetector(dictionary, params)


def mm_to_px(mm, dpi):
    return int(round(mm / 25.4 * dpi))


def write_png_with_dpi(path, image, dpi):
    Image.fromarray(image).save(path, dpi=(dpi, dpi))


def write_pdf_from_pngs(pdf_path, png_paths, dpi):
    pages = []
    for path in png_paths:
        page = Image.open(path).convert("RGB")
        pages.append(page)
    first, rest = pages[0], pages[1:]
    first.save(pdf_path, save_all=True, append_images=rest, resolution=dpi)


def load_font(size):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def draw_centered(draw, xy, text, font, fill=0):
    x, y = xy
    bbox = draw.textbbox((0, 0), text, font=font)
    width = bbox[2] - bbox[0]
    draw.text((x - width // 2, y), text, font=font, fill=fill)


def draw_labels(image, name, first_id, rows, cols, tag_mm, spacing_ratio, dpi,
                page_width_px, page_height_px, grid_width_px, grid_height_px,
                x0_page, y0_page):
    pil = Image.fromarray(image)
    draw = ImageDraw.Draw(pil)
    title_font = load_font(mm_to_px(4.2, dpi))
    head_font = load_font(mm_to_px(3.1, dpi))
    body_font = load_font(mm_to_px(2.4, dpi))
    small_font = load_font(mm_to_px(1.8, dpi))

    last_id = first_id + rows * cols - 1
    center_x = page_width_px // 2
    top_y = mm_to_px(10, dpi)
    draw_centered(draw, (center_x, top_y), f"{name}  AprilGrid 6x6  (tag36h11)", title_font)
    draw_centered(draw, (center_x, top_y + mm_to_px(7, dpi)), f"TAG IDs: {first_id} - {last_id}", head_font)
    draw_centered(
        draw,
        (center_x, top_y + mm_to_px(13, dpi)),
        f"TOP ROW: {last_id - cols + 1}-{last_id}    BOTTOM ROW: {first_id}-{first_id + cols - 1}",
        body_font,
    )

    arrow_x = page_width_px - mm_to_px(25, dpi)
    arrow_y0 = y0_page - mm_to_px(12, dpi)
    arrow_y1 = y0_page - mm_to_px(3, dpi)
    draw.line((arrow_x, arrow_y1, arrow_x, arrow_y0), fill=0, width=max(2, mm_to_px(0.5, dpi)))
    draw.polygon(
        [(arrow_x, arrow_y0 - mm_to_px(2.5, dpi)),
         (arrow_x - mm_to_px(2.2, dpi), arrow_y0 + mm_to_px(1.5, dpi)),
         (arrow_x + mm_to_px(2.2, dpi), arrow_y0 + mm_to_px(1.5, dpi))],
        fill=0,
    )
    draw.text((arrow_x + mm_to_px(3, dpi), arrow_y0 - mm_to_px(3, dpi)), "UP / +Z", font=body_font, fill=0)

    border_pad = mm_to_px(4, dpi)
    draw.rectangle(
        (x0_page - border_pad, y0_page - border_pad,
         x0_page + grid_width_px + border_pad, y0_page + grid_height_px + border_pad),
        outline=0,
        width=max(1, mm_to_px(0.25, dpi)),
    )

    info_y = y0_page + grid_height_px + mm_to_px(9, dpi)
    draw_centered(draw, (center_x, info_y), f"tagSize = {tag_mm:g} mm = {tag_mm / 1000.0:.3f} m    tagSpacing = {spacing_ratio}", body_font)
    draw_centered(draw, (center_x, info_y + mm_to_px(5, dpi)), "PRINT AT 100% / Actual Size - DO NOT use Fit to Page / scaling.", body_font)
    draw_centered(draw, (center_x, info_y + mm_to_px(10, dpi)), "After printing, MEASURE one black tag edge and use that value for --tag-size.", small_font)
    draw_centered(draw, (center_x, info_y + mm_to_px(15, dpi)), "Mount with this page's UP arrow pointing to world +Z.", small_font)

    ruler_mm = 50.0
    ruler_w = mm_to_px(ruler_mm, dpi)
    ruler_y = page_height_px - mm_to_px(18, dpi)
    ruler_x0 = center_x - ruler_w // 2
    ruler_x1 = ruler_x0 + ruler_w
    draw.line((ruler_x0, ruler_y, ruler_x1, ruler_y), fill=0, width=max(1, mm_to_px(0.35, dpi)))
    tick_h = mm_to_px(2.5, dpi)
    draw.line((ruler_x0, ruler_y - tick_h, ruler_x0, ruler_y + tick_h), fill=0, width=max(1, mm_to_px(0.35, dpi)))
    draw.line((ruler_x1, ruler_y - tick_h, ruler_x1, ruler_y + tick_h), fill=0, width=max(1, mm_to_px(0.35, dpi)))
    draw_centered(draw, (center_x, ruler_y + mm_to_px(3, dpi)), "reference: this line = 50 mm if printed at 100%", small_font)
    return np.array(pil)


def generate_board(name, first_id, rows, cols, tag_mm, spacing_ratio, margin_mm,
                   page_width_mm, page_height_mm, dpi, output_dir):
    dictionary, detector = make_detector()
    tag_px = mm_to_px(tag_mm, dpi)
    gap_px = mm_to_px(tag_mm * spacing_ratio, dpi)
    margin_px = mm_to_px(margin_mm, dpi)
    page_width_px = mm_to_px(page_width_mm, dpi)
    page_height_px = mm_to_px(page_height_mm, dpi)
    grid_width_px = cols * tag_px + (cols - 1) * gap_px
    grid_height_px = rows * tag_px + (rows - 1) * gap_px

    if grid_width_px + 2 * margin_px > page_width_px or grid_height_px + 2 * margin_px > page_height_px:
        raise ValueError(
            f"{name} does not fit A4: grid={grid_width_px}x{grid_height_px}px, "
            f"margin={margin_px}px, page={page_width_px}x{page_height_px}px. "
            "Reduce --tag-mm or --margin-mm."
        )

    image = np.full((page_height_px, page_width_px), 255, dtype=np.uint8)
    x0_page = (page_width_px - grid_width_px) // 2
    y0_page = (page_height_px - grid_height_px) // 2

    for board_row in range(rows):
        for col in range(cols):
            tag_id = first_id + board_row * cols + col
            image_row = rows - 1 - board_row
            y = y0_page + image_row * (tag_px + gap_px)
            x = x0_page + col * (tag_px + gap_px)
            image[y:y + tag_px, x:x + tag_px] = make_marker(dictionary, tag_id, tag_px)

    image = draw_labels(
        image, name, first_id, rows, cols, tag_mm, spacing_ratio, dpi,
        page_width_px, page_height_px, grid_width_px, grid_height_px, x0_page, y0_page
    )

    path = os.path.join(
        output_dir,
        f"A4_aprilgrid_6x6_tag36h11_{name}_ids{first_id:03d}_{first_id + rows * cols - 1:03d}_tag{tag_mm:.1f}mm.png"
    )
    write_png_with_dpi(path, image, dpi)

    corners, ids, _ = detector.detectMarkers(image)
    detected = sorted(int(v) for v in ids.flatten()) if ids is not None else []
    unique_detected = sorted(set(detected))
    duplicates = [(tag_id, count) for tag_id, count in Counter(detected).items() if count > 1]
    expected = list(range(first_id, first_id + rows * cols))
    ok = unique_detected == expected
    print(f"{name}: {path}")
    print(
        f"  A4 {page_width_mm:g}x{page_height_mm:g}mm @ {dpi}dpi, "
        f"tag={tag_mm:g}mm ({tag_px}px), gap={tag_mm * spacing_ratio:g}mm ({gap_px}px), "
        f"grid={tag_mm * (cols + (cols - 1) * spacing_ratio):.1f}mm"
    )
    print(
        f"  ids: {first_id}..{first_id + rows * cols - 1}, "
        f"unique_detect={len(unique_detected)}/{rows * cols} {'OK' if ok else 'CHECK'}"
    )
    if duplicates:
        print(f"  duplicate detections ignored by extractor: {duplicates}")
    if not ok:
        missing = sorted(set(expected) - set(unique_detected))
        extra = sorted(set(unique_detected) - set(expected))
        print(f"  missing={missing[:12]} extra={extra[:12]}")
    return path


def main():
    parser = argparse.ArgumentParser(description="Generate 4 unique AprilGrid boards for Edie GT")
    parser.add_argument("--output-dir", default=os.path.expanduser("~/ros2_ws/src"))
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--cols", type=int, default=6)
    parser.add_argument("--tag-mm", type=float, default=24.0,
                        help="printed black tag side length in millimeters; default fits A4 with 15mm margins")
    parser.add_argument("--spacing-ratio", type=float, default=0.3)
    parser.add_argument("--margin-mm", type=float, default=15.0)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--page", choices=("a4-portrait", "a4-landscape"), default="a4-portrait")
    parser.add_argument("--pdf-output", help="multi-page PDF output path; default: <output-dir>/aprilgrid_6x6_tag36h11_A4.pdf")
    args = parser.parse_args()

    page_width_mm, page_height_mm = (210.0, 297.0)
    if args.page == "a4-landscape":
        page_width_mm, page_height_mm = page_height_mm, page_width_mm

    os.makedirs(args.output_dir, exist_ok=True)
    print("Generating AprilTag36h11 boards with unique IDs")
    print(f"output_dir={args.output_dir}")
    print(
        f"page={args.page} ({page_width_mm:g}x{page_height_mm:g}mm), "
        f"dpi={args.dpi}, tag_mm={args.tag_mm:g}, spacing_ratio={args.spacing_ratio}, margin_mm={args.margin_mm:g}"
    )
    print()

    png_paths = []
    for name, first_id in BOARDS:
        path = generate_board(
            name, first_id, args.rows, args.cols, args.tag_mm, args.spacing_ratio,
            args.margin_mm, page_width_mm, page_height_mm, args.dpi, args.output_dir
        )
        png_paths.append(path)

    pdf_output = os.path.expanduser(args.pdf_output) if args.pdf_output else os.path.join(
        args.output_dir, "aprilgrid_6x6_tag36h11_A4.pdf"
    )
    write_pdf_from_pngs(pdf_output, png_paths, args.dpi)
    print(f"PDF: {pdf_output} (4 pages, same layout as PNG files)")

    print()
    print("Print at 100% / Actual size / no scaling. Do not use 'fit to page'.")
    print(f"If printed correctly, pass --tag-size {args.tag_mm / 1000.0:.3f} to gt_apriltag_4board.py.")
    print("Still measure one printed tag side with a ruler and use the measured value if it differs.")


if __name__ == "__main__":
    main()
