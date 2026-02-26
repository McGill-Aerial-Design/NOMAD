#!/usr/bin/env python3
"""
Convert YOLO26 .pt model to ONNX format for ZED SDK Custom Object Detection.

The ZED SDK takes an ONNX model and automatically converts it to a TensorRT engine
on first run (cached for subsequent launches). This script handles the .pt -> .onnx
conversion step.

Usage:
    python scripts/build/convert_yolo_to_onnx.py
    python scripts/build/convert_yolo_to_onnx.py --input yolo26.pt --output config/models/best.onnx --imgsz 512
    python scripts/build/convert_yolo_to_onnx.py --input yolo26.pt --output best.onnx --imgsz 512 --simplify

Requirements:
    pip install ultralytics onnx onnxsim (optional for simplify)
"""

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Convert YOLO .pt model to ONNX for ZED SDK custom object detection."
    )
    parser.add_argument(
        "--input", "-i",
        type=str,
        default="yolo26.pt",
        help="Path to YOLO .pt model file (default: yolo26.pt)",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="config/models/best.onnx",
        help="Output ONNX file path (default: config/models/best.onnx)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=512,
        help="Input image size for the model (default: 512, must match custom_onnx_input_size in config)",
    )
    parser.add_argument(
        "--simplify",
        action="store_true",
        help="Simplify ONNX model with onnxsim (optional, can improve TensorRT conversion)",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17, ZED SDK 4.x supports 11-17)",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Export with FP16 weights (for Jetson GPU inference)",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        print(f"ERROR: Input model not found: {input_path}")
        sys.exit(1)

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics package not installed.")
        print("Install with: pip install ultralytics")
        sys.exit(1)

    print(f"Loading YOLO model: {input_path}")
    model = YOLO(str(input_path))

    # Print model info
    print(f"Model classes ({len(model.names)}):")
    for class_id, class_name in model.names.items():
        print(f"  {class_id}: {class_name}")

    print(f"\nExporting to ONNX:")
    print(f"  Output: {output_path}")
    print(f"  Image size: {args.imgsz}x{args.imgsz}")
    print(f"  Opset: {args.opset}")
    print(f"  FP16: {args.half}")
    print(f"  Simplify: {args.simplify}")

    # Export to ONNX using ultralytics built-in exporter
    export_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=args.simplify,
        half=args.half,
    )

    # Move to desired output path if different from export default
    export_path = Path(export_path)
    if export_path != output_path:
        import shutil
        shutil.move(str(export_path), str(output_path))
        print(f"\nMoved ONNX model to: {output_path}")
    else:
        print(f"\nONNX model saved to: {output_path}")

    # Verify the output
    if output_path.exists():
        size_mb = output_path.stat().st_size / (1024 * 1024)
        print(f"ONNX model size: {size_mb:.1f} MB")
        print("\nConversion successful.")
        print("\nNext steps:")
        print("  1. Copy the ONNX model to the Jetson:")
        print(f"     scp {output_path} mad@100.85.121.98:~/NOMAD/config/models/best.onnx")
        print("  2. The ZED SDK will auto-convert to TensorRT on first launch")
        print("     (this takes several minutes on Jetson Orin Nano)")
        print("  3. Subsequent launches will use the cached TensorRT engine")
    else:
        print("ERROR: ONNX file was not created.")
        sys.exit(1)


if __name__ == "__main__":
    main()
