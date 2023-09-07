import cv2
import numpy as np
import matplotlib.pyplot as plt

def load_image(image_path):
    return cv2.imread(image_path)[:, :, ::-1]

def draw_epipolar_lines(
    F: np.ndarray,
    img_left: np.ndarray,
    img_right: np.ndarray,
    pts_left: np.ndarray,
    pts_right: np.ndarray,
    figsize=(10, 8),
):
    """Draw the epipolar lines given the fundamental matrix, left right images
    and left right datapoints

    Args:
        F: a 3 x 3 numpy array representing the fundamental matrix, such that
            p_right^T @ F @ p_left = 0 for correct correspondences
        img_left: array representing image 1.
        img_right: array representing image 2.
        pts_left: array of shape (N,2) representing image 1 datapoints.
        pts_right: array of shape (N,2) representing image 2 datapoints.

    Returns:
        None
    """
    # ------------ lines in the RIGHT image --------------------
    imgh_right, imgw_right = img_right.shape[:2]
    # corner points, as homogeneous (x,y,1)
    p_ul = np.asarray([0, 0, 1])
    p_ur = np.asarray([imgw_right, 0, 1])
    p_bl = np.asarray([0, imgh_right, 1])
    p_br = np.asarray([imgw_right, imgh_right, 1])

    # The equation of the line through two points
    # can be determined by taking the ‘cross product’
    # of their homogeneous coordinates.

    # left and right border lines, for the right image
    l_l = np.cross(p_ul, p_bl)
    l_r = np.cross(p_ur, p_br)

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=figsize)

    ax[1].imshow(img_right)
    ax[1].autoscale(False)
    ax[1].scatter(
        pts_right[:, 0], pts_right[:, 1], marker="o", s=20, c="yellow", edgecolors="black"
    )
    for p in pts_left:
        p = np.hstack((p, 1))[:, np.newaxis]
        # get defn of epipolar line in right image, corresponding to left point p
        l_e = np.dot(F, p).squeeze()
        # find where epipolar line in right image crosses the left and image borders
        p_l = np.cross(l_e, l_l)
        p_r = np.cross(l_e, l_r)
        # convert back from homogeneous to cartesian by dividing by 3rd entry
        # draw line between point on left border, and on the right border
        x = [p_l[0] / p_l[2], p_r[0] / p_r[2]]
        y = [p_l[1] / p_l[2], p_r[1] / p_r[2]]
        ax[1].plot(x, y, linewidth=1, c="brown")

    # ------------ lines in the LEFT image --------------------
    imgh_left, imgw_left = img_left.shape[:2]

    # corner points, as homogeneous (x,y,1)
    p_ul = np.asarray([0, 0, 1])
    p_ur = np.asarray([imgw_left, 0, 1])
    p_bl = np.asarray([0, imgh_left, 1])
    p_br = np.asarray([imgw_left, imgh_left, 1])

    # left and right border lines, for left image
    l_l = np.cross(p_ul, p_bl)
    l_r = np.cross(p_ur, p_br)

    ax[0].imshow(img_left)
    ax[0].autoscale(False)
    ax[0].scatter(
        pts_left[:, 0], pts_left[:, 1], marker="o", s=20, c="yellow", edgecolors="black"
    )
    for p in pts_right:
        p = np.hstack((p, 1))[:, np.newaxis]
        # defn of epipolar line in the left image, corresponding to point p in the right image
        l_e = np.dot(F.T, p).squeeze()
        p_l = np.cross(l_e, l_l)
        p_r = np.cross(l_e, l_r)
        x = [p_l[0] / p_l[2], p_r[0] / p_r[2]]
        y = [p_l[1] / p_l[2], p_r[1] / p_r[2]]
        ax[0].plot(x, y, linewidth=1, c="brown")
    plt.title("epipolar lines")
    plt.show()


def get_matches(
    pic_a: np.ndarray, pic_b: np.ndarray, n_feat: int
) -> (np.ndarray, np.ndarray):

    img1 = cv2.cvtColor(pic_a, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(pic_b, cv2.COLOR_BGR2GRAY)

    MIN_MATCH_COUNT = 7
    # Initiate SIFT detector with custom parameters
    sift = cv2.SIFT_create(nfeatures=n_feat)
    
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 100)  # Increase the number of checks
    
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    good = []
    # Lowe's ratio test with a different threshold
    for m,n in matches:
        if m.distance < 0.9 * n.distance:  
            good.append(m)
            
    if len(good) > MIN_MATCH_COUNT:
        img1_pts = np.float32([ kp1[m.queryIdx].pt for m in good ])
        img2_pts = np.float32([ kp2[m.trainIdx].pt for m in good ])
        
        # RANSAC for outlier removal
        _, mask = cv2.findHomography(img1_pts, img2_pts, cv2.RANSAC, 5.0)
        img1_pts = img1_pts[mask.ravel() == 1]
        img2_pts = img2_pts[mask.ravel() == 1]
    
    return img1_pts, img2_pts


def show_correspondence2(
    imgA: np.ndarray,
    imgB: np.ndarray,
    X1: np.ndarray,
    Y1: np.ndarray,
    X2: np.ndarray,
    Y2: np.ndarray,
    line_colors=None,
) -> None:
    """Visualizes corresponding points between two images. Corresponding points
    will have the same random color.

    Args:
        imgA: a numpy array representing image 1.
        imgB: a numpy array representing image 2.
        X1: a numpy array representing x coordinates of points from image 1.
        Y1: a numpy array representing y coordinates of points from image 1.
        X2: a numpy array representing x coordinates of points from image 2.
        Y2: a numpy array representing y coordinates of points from image 2.
        line_colors: a N x 3 numpy array containing colors of correspondence
            lines (optional)

    Returns:
        None
    """
    newImg = hstack_images(imgA, imgB)
    shiftX = imgA.shape[1]
    X1 = X1.astype(int)
    Y1 = Y1.astype(int)
    X2 = X2.astype(int)
    Y2 = Y2.astype(int)

    dot_colors = np.random.rand(len(X1), 3)
    if imgA.dtype == np.uint8:
        dot_colors *= 255
    if line_colors is None:
        line_colors = dot_colors

    for x1, y1, x2, y2, dot_color, line_color in zip(
        X1, Y1, X2, Y2, dot_colors, line_colors
    ):
        newImg = cv2.circle(newImg, (x1, y1), 5, dot_color, -1)
        newImg = cv2.circle(newImg, (x2 + shiftX, y2), 5, dot_color, -1)
        newImg = cv2.line(
            newImg, (x1, y1), (x2 + shiftX, y2), line_color, 2, cv2.LINE_AA
        )

    return newImg

def hstack_images(imgA: np.ndarray, imgB: np.ndarray) -> np.ndarray:
    """Stacks 2 images side-by-side

    Args:
        imgA: a numpy array representing image 1.
        imgB: a numpy array representing image 2.

    Returns:
        img: a numpy array representing the images stacked side by side.
    """
    Height = max(imgA.shape[0], imgB.shape[0])
    Width = imgA.shape[1] + imgB.shape[1]

    newImg = np.zeros((Height, Width, 3), dtype=imgA.dtype)
    newImg[: imgA.shape[0], : imgA.shape[1], :] = imgA
    newImg[: imgB.shape[0], imgA.shape[1] :, :] = imgB

    return newImg

