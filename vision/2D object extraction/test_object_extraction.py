import json
import os
from datetime import datetime

import cv2
import numpy as np

from object_extraction_from_image import extract_objects, slic_objects


def draw_objects(image: np.ndarray, objects: list) -> np.ndarray:
    vis = image.copy()
    for idx, obj in enumerate(objects):
        x, y, w, h = obj["bounding_box"]
        cx, cy = obj["centroid"]
        area = obj["area"]
        aspect_ratio = obj.get("aspect_ratio", 0.0)

        # Choose a color deterministically by index
        color = tuple(int(v) for v in np.random.default_rng(idx).integers(0, 255, size=3))

        cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
        cv2.circle(vis, (int(cx), int(cy)), 3, (0, 255, 0), -1)

        label = f"ID:{idx} area:{int(area)} ar:{aspect_ratio:.2f}"
        cv2.putText(vis, label, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    # Also text summary of how many objects
    cv2.putText(vis, f"Objects: {len(objects)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return vis


def compute_binary_mask(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask


def resize_keep_aspect(image: np.ndarray, max_dim: int) -> np.ndarray:
    """Resize image so that max(width, height) == max_dim, keeping aspect ratio.
    If image is already smaller than max_dim, return unchanged.
    Set max_dim to 0/None to disable.
    """
    if not max_dim or max_dim <= 0:
        return image
    h, w = image.shape[:2]
    current_max = max(h, w)
    if current_max <= max_dim:
        return image
    scale = max_dim / float(current_max)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)


def main():
    # Select method: "extract" (threshold+contours) or "slic" (superpixels)
    METHOD = "slic"  # change to "slic" to test SLIC superpixels

    # SLIC parameters (used only when METHOD == "slic")
    SLIC_REGION_SIZE = 20
    SLIC_RULER = 10.0
    SLIC_ITERATIONS = 10

    # Resize control: set maximum dimension for faster processing (set to 0 to disable)
    RESIZE_MAX_DIM = 80

    # Specify your image path here (edit this line)
    IMAGE_PATH = "/Users/irakli/Desktop/Minerva/Visual processing/2D object extraction/picture of a kitchen.png"

    # Outputs will be saved alongside this script in 'outputs/'
    SCRIPT_DIR = os.path.dirname(__file__)
    OUT_DIR = os.path.join(SCRIPT_DIR, "outputs")
    os.makedirs(OUT_DIR, exist_ok=True)

    image = cv2.imread(IMAGE_PATH)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

    # Optionally downscale image for speed while keeping aspect ratio
    orig_h, orig_w = image.shape[:2]
    image = resize_keep_aspect(image, RESIZE_MAX_DIM)
    proc_h, proc_w = image.shape[:2]
    if (proc_w, proc_h) != (orig_w, orig_h):
        print(f"Resized from {orig_w}x{orig_h} to {proc_w}x{proc_h} (max_dim={RESIZE_MAX_DIM})")

    # Run selected extraction
    if METHOD == "slic":
        objects = slic_objects(
            image,
            region_size=SLIC_REGION_SIZE,
            ruler=SLIC_RULER,
            iterations=SLIC_ITERATIONS,
        )
    else:
        objects = extract_objects(image)

    # Draw visualization
    vis = draw_objects(image, objects)

    # Also compute and save the binary/boundary mask to show what counts as object
    if METHOD == "slic":
        slic = cv2.ximgproc.createSuperpixelSLIC(image, region_size=SLIC_REGION_SIZE, ruler=SLIC_RULER)
        slic.iterate(SLIC_ITERATIONS)
        mask = slic.getLabelContourMask()
    else:
        mask = compute_binary_mask(image)

    # Build output stems
    base = os.path.splitext(os.path.basename(IMAGE_PATH))[0]
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    timestamp = 0 # this is to overwrite the previous outputs, as opposed to creating a new file all the time
    vis_path = os.path.join(OUT_DIR, f"{base}_{METHOD}_objects_{timestamp}.png")
    json_path = os.path.join(OUT_DIR, f"{base}_{METHOD}_objects_{timestamp}.json")
    mask_path = os.path.join(OUT_DIR, f"{base}_{METHOD}_mask_{timestamp}.png")

    # Save outputs
    cv2.imwrite(vis_path, vis)
    cv2.imwrite(mask_path, mask)
    with open(json_path, "w") as f:
        json.dump({
            "image": IMAGE_PATH,
            "num_objects": len(objects),
            "objects": objects,
        }, f, indent=2)

    print(f"Saved visualization: {vis_path}")
    print(f"Saved binary mask: {mask_path}")
    print(f"Saved objects JSON: {json_path}")


if __name__ == "__main__":
    main()


