import cv2
import torch
import clip
import numpy as np
from PIL import Image
import time
# ===== MobileSAM =====
from mobile_sam import sam_model_registry, SamAutomaticMaskGenerator

# ------------------------------
# 配置
# ------------------------------
IMAGE_PATH = "/workspace/unet-pytorch/VOCdevkit_hdu/VOC2007/JPEGImages/building2_lawn_214.jpg"
SAM_CKPT = "weights/mobile_sam.pt"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# 类别映射
CLASSES = {
    "cement_road": "concrete road or cement road or asphalt road",
    "tree": "tree",
    "grass": "grass or lawn",
    "other": "background or other objects"
}

# 颜色映射
color_map = {
    "cement_road": (160, 160, 160),  # 灰色
    "tree": (0, 128, 0),             # 深绿
    "grass": (0, 255, 0),            # 亮绿
    "other": (0, 0, 0)               # 黑色
}

# ------------------------------
# 加载模型
# ------------------------------
def load_models():
    # MobileSAM
    sam = sam_model_registry["vit_t"](checkpoint=SAM_CKPT)
    sam.to(device=DEVICE)
    sam.eval()

    mask_generator = SamAutomaticMaskGenerator(
        sam,
        points_per_side=32,
        pred_iou_thresh=0.88,
        stability_score_thresh=0.92,
        min_mask_region_area=500
    )

    # CLIP
    clip_model, preprocess = clip.load("ViT-B/32", device=DEVICE)
    clip_model.eval()

    return mask_generator, clip_model, preprocess

# ------------------------------
# CLIP 分类
# ------------------------------
def classify_with_clip(crop, clip_model, preprocess, text_features):
    image = preprocess(crop).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        image_features = clip_model.encode_image(image)
        image_features /= image_features.norm(dim=-1, keepdim=True)

        similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)

    score, idx = similarity[0].max(0)
    return idx.item(), score.item()

# ------------------------------
# 主流程
# ------------------------------
def main():
    # 读取图像

    image_bgr = cv2.imread(IMAGE_PATH)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    H, W, _ = image_rgb.shape

    # 保存原图
    cv2.imwrite("original.png", image_bgr)

    mask_generator, clip_model, preprocess = load_models()

    # CLIP 文本特征
    text_prompts = list(CLASSES.values())
    text_tokens = clip.tokenize(text_prompts).to(DEVICE)

    with torch.no_grad():
        text_features = clip_model.encode_text(text_tokens)
        text_features /= text_features.norm(dim=-1, keepdim=True)

    # MobileSAM 生成 mask
    start_time = time.time()
    masks = mask_generator.generate(image_rgb)
    print(f"Generated {len(masks)} masks")

    # 输出缓存
    final_mask_id = np.zeros((H, W), dtype=np.uint8)
    mask_color_vis = np.zeros_like(image_bgr)
    overlay_vis = image_bgr.copy()

    # 遍历 mask
    for m in masks:
        seg = m["segmentation"]
        if seg.sum() < 800:
            continue

        ys, xs = np.where(seg)
        x1, x2 = xs.min(), xs.max()
        y1, y2 = ys.min(), ys.max()

        crop = image_rgb[y1:y2, x1:x2]
        if crop.size == 0:
            continue

        crop_pil = Image.fromarray(crop)

        cls_idx, score = classify_with_clip(
            crop_pil, clip_model, preprocess, text_features
        )

        cls_name = list(CLASSES.keys())[cls_idx]

        # 置信度低的 mask 归为其他
        if score < 0.6:
            cls_name = "other"
            cls_idx = list(CLASSES.keys()).index("other")

        color = color_map[cls_name]

        # 纯 mask 可视化
        mask_color_vis[seg] = color

        # overlay
        overlay_vis[seg] = (
            0.6 * overlay_vis[seg] + 0.4 * np.array(color)
        ).astype(np.uint8)

        # ID mask
        final_mask_id[seg] = cls_idx + 1

        print(f"Detected {cls_name}, score={score:.2f}")
    end_time = time.time()

    # 保存结果
    cv2.imwrite("mask_color.png", mask_color_vis)
    cv2.imwrite("mask_overlay.png", overlay_vis)
    cv2.imwrite("mask_id.png", final_mask_id)
    
    total_time = end_time - start_time
    print("Saved files:")
    print(" - original.png")
    print(" - mask_color.png")
    print(" - mask_overlay.png")
    print(" - mask_id.png")
    print(f"Total processing time: {total_time:.2f} seconds")

if __name__ == "__main__":
    main()
