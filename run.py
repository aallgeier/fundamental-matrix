import cv2 
import numpy as np
import matplotlib.pyplot as plt
from src.estimate_fundamental_matrix import estimate_fundamental_matrix
from src.ransac import ransac_fundamental_matrix 
from src.utils import get_matches
from src.utils import draw_epipolar_lines
from src.utils import load_image
from src.utils import show_correspondence2

pic_a = load_image("data/img1.jpg")
scale_a = 0.5
pic_b = load_image("data/img2.jpg")
scale_b = 0.5
n_feat = 4000

pic_a = cv2.resize(pic_a, None, fx=scale_a, fy=scale_a)
pic_b = cv2.resize(pic_b, None, fx=scale_b, fy=scale_b)

points_2d_pic_a, points_2d_pic_b = get_matches(pic_a, pic_b, n_feat)
print("Found {:d} possibly matching features".format(len(points_2d_pic_a)))

F, matched_points_a, matched_points_b = ransac_fundamental_matrix(
    points_2d_pic_a, points_2d_pic_b
)

# Draw the epipolar lines on the images and corresponding matches
match_image = show_correspondence2(
    pic_a,
    pic_b,
    matched_points_a[:, 0],
    matched_points_a[:, 1],
    matched_points_b[:, 0],
    matched_points_b[:, 1],
)
plt.figure()
plt.title("OpenCV SIFT matches")
plt.imshow(match_image)
plt.tight_layout()
plt.show()

draw_epipolar_lines(
    F, pic_a, pic_b, matched_points_a, matched_points_b, figsize=(12, 8)
)

















