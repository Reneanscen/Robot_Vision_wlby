import torch
import torchvision.transforms as transforms
import torchvision.models as models
from PIL import Image
import numpy as np


model = torch.hub.load('pytorch/vision', 'resnet18', pretrained=True)
# model = models.resnet18(pretrained=True)
model.eval()
def load_and_preprocess_image(image_path):
    # 加载图像并预处理
    img = Image.open(image_path).convert("RGB")
    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])
    img_tensor = transform(img).unsqueeze(0)
    return img_tensor

def extract_features(image_path):
    # 使用ResNet18模型提取图像特征向量

    with torch.no_grad():
        img_tensor = load_and_preprocess_image(image_path)
        features = model(img_tensor)
    return features.flatten().numpy()

def cosine_similarity(vec1, vec2):
    # 计算余弦相似度
    dot_product = np.dot(vec1, vec2)
    norm1 = np.linalg.norm(vec1)
    norm2 = np.linalg.norm(vec2)
    similarity = dot_product / (norm1 * norm2)
    return similarity

def match_left_and_right_shoes_with_similarity(bounding_boxes, image_paths):
    pairs = []

    for i in range(len(image_paths)):
        features1 = extract_features(image_paths[i])
        matched_box = None
        max_similarity = -1

        for j in range(i + 1, len(image_paths)):
            features2 = extract_features(image_paths[j])
            similarity = cosine_similarity(features1, features2)
            print(similarity)

            if similarity > max_similarity:
                matched_box = j
                max_similarity = similarity

        if matched_box is not None and max_similarity>0.65:
            pairs.append((image_paths[i], image_paths[matched_box]))
        else:
            pairs.append((image_paths[i],))
    return pairs


    # pairs = []
    # unmatched_boxes = bounding_boxes.copy()
    #
    # while len(unmatched_boxes) >= 2:
    #     current_box = unmatched_boxes[0]
    #     matched_box = None
    #     max_similarity = -1
    #
    #     for box in unmatched_boxes[1:]:
    #         idx1 = bounding_boxes.index(current_box)
    #         idx2 = bounding_boxes.index(box)
    #         img_path1, img_path2 = image_paths[idx1], image_paths[idx2]
    #         features1, features2 = extract_features(img_path1), extract_features(img_path2)
    #         similarity = cosine_similarity(features1, features2)
    #
    #         if similarity > max_similarity:
    #             matched_box = box
    #             max_similarity = similarity
    #
    #     if matched_box:
    #         pairs.append((current_box, matched_box))
    #         unmatched_boxes.remove(current_box)
    #         unmatched_boxes.remove(matched_box)
    #     else:
    #         # 如果没有找到匹配的box，则认为当前box是单只鞋子
    #         pairs.append((current_box,))
    #         unmatched_boxes.remove(current_box)
    #
    # return pairs

# 示例用法
bounding_boxes = [
    (10, 20, 50, 70),  # (x_min, y_min, x_max, y_max) for shoe 1
    (80, 30, 120, 80),  # (x_min, y_min, x_max, y_max) for shoe 2
    # 可能还有其他鞋子的bounding box
]

image_paths = [
    "/home/lz/yolov5/1.png",  # 图像1的路径
    "/home/lz/yolov5/2.png",  # 图像2的路径
    "/home/lz/yolov5/3.png",
    "/home/lz/yolov5/4.png",

    # 可能还有其他图像的路径
]

shoe_pairs = match_left_and_right_shoes_with_similarity(bounding_boxes, image_paths)
for index, pair in enumerate(shoe_pairs):
    if len(pair) == 2:
        print(f"第{index + 1}双鞋子的左鞋:", pair[0])
        print(f"第{index + 1}双鞋子的右鞋:", pair[1])
    else:
        print(f"第{index + 1}只鞋子:", pair[0])
