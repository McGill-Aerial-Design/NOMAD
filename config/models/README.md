# NOMAD Model Files

This directory holds ONNX/TensorRT model files for on-device inference.

## Files

- `best.onnx` -- YOLO26 circle detection model (converted from yolo26.pt)
  - Input: 1x3x512x512
  - Classes: 6 (black_circle, blue_circle, green_circle, red_circle, white_circle, yellow_circle)
  - Used by: ZED SDK custom object detection pipeline

## Generating the ONNX Model

```bash
python scripts/build/convert_yolo_to_onnx.py --input yolo26.pt --output config/models/best.onnx --imgsz 512
```

## Note

The actual `.onnx` and `.pt` files are not committed to git (see .gitignore).
Deploy the ONNX model to the Jetson manually after conversion.
