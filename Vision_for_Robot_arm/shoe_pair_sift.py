# import cv2
#
# def match_shoes(query_image, template_images):
#     # 加载查询图像
#     img_rgb = cv2.imread(query_image)
#     # img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
#
#     # 初始化匹配器
#     sift = cv2.SIFT_create()
#
#     # 寻找查询图像中的关键点和描述符
#     kp1, des1 = sift.detectAndCompute(img_rgb, None)
#
#     # 用于存储匹配结果的列表
#     matches = []
#
#     # 遍历每个模板图像
#     for template_image in template_images:
#         # 加载模板图像
#         template = cv2.imread(template_image, 0)
#
#         # 寻找模板图像中的关键点和描述符
#         kp2, des2 = sift.detectAndCompute(template, None)
#
#         # 使用FLANN匹配器进行特征匹配
#         matcher = cv2.FlannBasedMatcher()
#         knn_matches = matcher.knnMatch(des1, des2, k=2)
#
#         # 根据Lowe's Ratio Test处理匹配结果
#         good_matches = []
#         for m, n in knn_matches:
#             if m.distance < 0.7 * n.distance:
#                 good_matches.append(m)
#
#         # 记录匹配结果
#         matches.append((template_image, len(good_matches)))
#
#     # 对匹配结果进行排序
#     matches.sort(key=lambda x: x[1], reverse=True)
#
#     # 打印匹配结果
#     for match in matches:
#         print("Template: {}, Matches: {}".format(match[0], match[1]))
#
# # 示例用法
# query_image = '1.png'
# template_images = ['2.png', '3.png', '4.png', '5.jpeg', '6.jpeg', '7.jpeg']
# match_shoes(query_image, template_images)


import cv2
import numpy as np

def match_left_and_right_shoes(image_paths):
    pairs = []

    # 初始化SIFT特征提取器
    sift = cv2.SIFT_create()

    for image_path in image_paths:
        # 读取图像并查找特征点和描述子
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        kp, des = sift.detectAndCompute(image, None)

        # 在这里可以加入一些筛选条件，比如特征点数量、描述子匹配分数等

        # 将特征点和描述子保存到列表中
        pairs.append((kp, des, image_path))

    # 进行特征匹配
    bf = cv2.BFMatcher()
    for i, (_, des1, image_path1) in enumerate(pairs):
        for j, (_, des2, image_path2) in enumerate(pairs):
            if i != j:
                matches = bf.knnMatch(des1, des2, k=2)
                good_matches = []
                for m, n in matches:
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)
                if len(good_matches) >= 10:  # 调整匹配阈值或者特征点数量的条件
                    print(f"配对成功：{image_path1} 和 {image_path2}")

# 示例用法
image_paths = [
    "1.png",  # 图像1的路径
    "2.png",  # 图像2的路径
    "3.png",
    "4.png",
    # 可能还有其他图像的路径
]

match_left_and_right_shoes(image_paths)
