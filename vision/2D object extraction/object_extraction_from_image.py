import cv2
import numpy as np


def slic_objects(image, n_segments=200, region_size=20, ruler=10.0, iterations=10):
    slic = cv2.ximgproc.createSuperpixelSLIC(image, region_size=region_size, ruler=ruler)
    slic.iterate(iterations)
    labels = slic.getLabels()
    num_labels = slic.getNumberOfSuperpixels()

    objects = []
    for i in range(num_labels):
        ys, xs = np.where(labels == i)
        if xs.size == 0:
            continue

        # Bounding box
        x_min, x_max = int(xs.min()), int(xs.max())
        y_min, y_max = int(ys.min()), int(ys.max())
        w, h = x_max - x_min + 1, y_max - y_min + 1

        # Binary mask for the superpixel to compute moments/centroid robustly
        seg_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        seg_mask[ys, xs] = 255

        M = cv2.moments(seg_mask, binaryImage=True)
        if M["m00"] == 0:
            # Fallback centroid from mean indices
            cx, cy = int(xs.mean()), int(ys.mean())
        else:
            cx = int(M["m10"] / M["m00"]) 
            cy = int(M["m01"] / M["m00"])

        # Hu moments from binary mask moments
        hu = cv2.HuMoments(M).flatten().tolist()

        # Mean color (BGR)
        mean_color = image[ys, xs].mean(axis=0).tolist()

        objects.append({
            "centroid": (cx, cy),
            "area": int(xs.size),
            "bounding_box": (x_min, y_min, w, h),
            "hu_moments": hu,
            "aspect_ratio": (w / float(h)) if h > 0 else 0.0,
            "mean_color": mean_color[:3]
        })
    return objects


def extract_objects(image):
    # Convert to grayscale + threshold
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    objects = []
    for c in contours:
        M = cv2.moments(c)
        if M["m00"] == 0:  # avoid division by zero
            continue
        cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
        
        area = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        hu = cv2.HuMoments(M).flatten()
        mean_color = cv2.mean(image, mask=cv2.drawContours(np.zeros_like(gray), [c], -1, 255, -1))
        
        objects.append({
            "centroid": (cx, cy),
            "area": area,
            "bounding_box": (x, y, w, h),
            "hu_moments": hu.tolist(),
            "aspect_ratio": w / float(h),
            "mean_color": mean_color[:3]
        })
    return objects
