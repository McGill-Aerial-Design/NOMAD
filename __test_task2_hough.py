"""Test a Hough-circle approach with color-density gating."""
import cv2, numpy as np, math

IMG = r"C:\Users\Youssef\Desktop\NOMAD_20260520_230756.png"
img = cv2.imread(IMG, cv2.IMREAD_COLOR)
h, w = img.shape[:2]

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
s = hsv[:,:,1]; v = hsv[:,:,2]
a = lab[:,:,1].astype(np.int16); b = lab[:,:,2].astype(np.int16)
chroma = np.abs(a-128) + np.abs(b-128)
mauve = (a > 128) & (b < 135) & (chroma >= 4) & (v >= 70) & (v <= 245)
blue = (a < 130) & (b < 132) & (chroma >= 4) & (v >= 60) & (v <= 230)
not_white = ~((s <= 25) & (v >= 195))
target_raw = ((mauve | blue) & not_white).astype(np.uint8) * 255

backing_raw = ((s <= 40) & (v >= 175) & (chroma <= 22)).astype(np.uint8) * 255
k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
backing = cv2.morphologyEx(backing_raw, cv2.MORPH_CLOSE, k5, iterations=2)
backing = cv2.morphologyEx(backing, cv2.MORPH_OPEN, k5, iterations=1)

# --- approach A: Hough on grayscale ---
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray_b = cv2.medianBlur(gray, 5)
min_r = max(8, int(round(min(h, w) * 0.012)))
max_r = min(h, w) // 2

print("=== Hough on grayscale ===")
for p2 in [15, 20, 25, 30, 40, 50]:
    circles = cv2.HoughCircles(gray_b, cv2.HOUGH_GRADIENT, dp=1.2,
                               minDist=max(20, min_r*2), param1=80, param2=p2,
                               minRadius=min_r, maxRadius=max_r)
    nfound = 0 if circles is None else len(circles[0])
    rows = []
    if circles is not None:
        for cx, cy, r in circles[0]:
            cx, cy, r = int(cx), int(cy), int(r)
            yy, xx = np.ogrid[:h, :w]
            inside = (xx-cx)**2 + (yy-cy)**2 <= (r*0.85)**2
            ic = int(inside.sum()); ti = int(((target_raw>0) & inside).sum())
            density = ti / max(ic, 1)
            # ring of white-backing around the circle
            ring = ((xx-cx)**2 + (yy-cy)**2 >= (r*1.10)**2) & \
                   ((xx-cx)**2 + (yy-cy)**2 <= (r*1.85)**2)
            rc = int(ring.sum()); rb = int(((backing>0) & ring).sum())
            backing_ratio = rb / max(rc, 1)
            rows.append((density, backing_ratio, cx, cy, r))
    rows.sort(reverse=True)  # by density
    top = rows[:5]
    print(f"p2={p2:3d}: found={nfound:3d}, top by density:")
    for density, br, cx, cy, r in top:
        print(f"   center=({cx:3d},{cy:3d}) r={r:3d} density={density:.2f} backing_ratio={br:.2f}")
