import sys, os, cv2
sys.path.insert(0, os.path.abspath('.'))
from edge_core.task2_circle_verify import Task2CircleDetector
det = Task2CircleDetector(min_radius_px=8, max_radius_px=400)
for label, p in [
    ("purple-pink @ 0.5m (snap9)", r"C:\Users\Youssef\Desktop\NOMAD_snap9.jpg"),
    ("blue @ 1m (snap8)",          r"C:\Users\Youssef\Desktop\NOMAD_snap8.jpg"),
    ("blue @ 30cm (snap5)",        r"C:\Users\Youssef\Desktop\NOMAD_snap5.jpg"),
    ("warehouse mauve",            r"C:\Users\Youssef\Downloads\image.jpg"),
    ("empty wall snap2",           r"C:\Users\Youssef\Desktop\NOMAD_snap2.jpg"),
    ("empty wall snap4",           r"C:\Users\Youssef\Desktop\NOMAD_snap4.jpg"),
]:
    img = cv2.imread(p, cv2.IMREAD_COLOR)
    if img is None: continue
    # Match the bridge: downsample to 960
    h, w = img.shape[:2]
    if w > 960:
        img = cv2.resize(img, (960, int(h*960/w)), interpolation=cv2.INTER_AREA)
    cs = det.detect(img)
    print(f"-- {label}: {img.shape[1]}x{img.shape[0]} -- {len(cs)} dets")
    for c in cs[:3]:
        print(f"  r={c.radius} cx={c.cx} cy={c.cy} conf={c.confidence:.2f}")
