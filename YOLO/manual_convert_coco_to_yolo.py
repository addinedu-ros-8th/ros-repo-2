#!/usr/bin/env python3
# manual_convert_coco_to_yolo.py

import os
import json
from pathlib import Path
import yaml
from tqdm import tqdm

# 1) 경로 설정 — 실제 파일들이 있는 위치를 정확히 가리킵니다.
dataset_root   = Path("/home/yun/dev_ws/ros_yolo/datasets/coco-person/fiftyone/coco-2017")
annotations_dir = dataset_root / "raw"                # JSON 어노테이션이 있는 폴더
images_train    = dataset_root / "train" / "data"     # train 이미지
images_val      = dataset_root / "validation" / "data"  # val 이미지
# 2) 출력할 YOLOv5/YOLOv8 형식 디렉토리
output_root     = Path("/home/yun/dev_ws/ros_yolo/datasets/coco-person-yolo")
train_output    = output_root / "train"
val_output      = output_root / "val"

# 3) 필요한 폴더 만들기
for folder in [
    train_output / "images", train_output / "labels",
    val_output   / "images", val_output   / "labels"
]:
    folder.mkdir(parents=True, exist_ok=True)

def convert_split(split_name, images_dir, ann_json):
    """
    split_name: "train" or "validation"
    images_dir: 해당 split의 이미지 폴더(Path)
    ann_json:    instances_*.json 파일(Path)
    """
    # a) COCO JSON 불러오기
    with open(ann_json, 'r') as f:
        coco = json.load(f)

    # b) 이미지 메타맵: id → (filename, width, height)
    img_meta = {
        i["id"]: (i["file_name"], i["width"], i["height"])
        for i in coco["images"]
    }

    # c) 사람(person=category_id 1) annotation만 필터
    anns = [a for a in coco["annotations"] if a["category_id"] == 1]

    # d) 이미지별로 묶기
    per_image = {}
    for a in anns:
        per_image.setdefault(a["image_id"], []).append(a["bbox"])

    # e) YOLO 포맷으로 변환 & 파일/링크 생성
    out = train_output if split_name == "train" else val_output
    for img_id, bboxes in tqdm(per_image.items(), desc=f"Converting {split_name}"):
        fname, w, h = img_meta[img_id]
        src = images_dir / fname
        dst_img = out / "images" / fname
        if src.exists() and not dst_img.exists():
            os.link(src, dst_img)  # 하드링크로 공간 절약

        # 라벨 파일 쓰기
        lf = (out / "labels" / fname).with_suffix(".txt")
        with open(lf, "w") as f:
            for x, y, bw, bh in bboxes:
                xc = (x + bw/2) / w
                yc = (y + bh/2) / h
                nw = bw / w
                nh = bh / h
                f.write(f"0 {xc:.6f} {yc:.6f} {nw:.6f} {nh:.6f}\n")

# 4) train/validation 변환 호출
convert_split(
    "train",
    images_train,
    annotations_dir / "instances_train2017.json"
)
convert_split(
    "validation",
    images_val,
    annotations_dir / "instances_val2017.json"
)

# 5) data.yaml 생성
cfg = {
    "train": str(train_output / "images"),
    "val":   str(val_output   / "images"),
    "nc": 1,
    "names": ["person"]
}
with open("/home/yun/dev_ws/ros_yolo/data.yaml", "w") as f:
    yaml.dump(cfg, f)

print("✅ Conversion complete.")
print("→ data.yaml written at /home/yun/dev_ws/ros_yolo/data.yaml")
