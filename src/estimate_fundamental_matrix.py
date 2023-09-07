import numpy as np

def normalize_points(points):
    """
    Perform coordinate normalization through linear transformations.
    Args:
        points: A numpy array of shape (N, 2) representing the 2D points in
            the image

    Returns:
        points_normalized: A numpy array of shape (N, 2) representing the
            normalized 2D points in the image
        T: transformation matrix representing the product of the scale and
            offset matrices
    """

    c_u = np.mean(points[:, 0])
    c_v = np.mean(points[:, 1])

    s_u = np.std(points[:, 0])
    s_v = np.std(points[:, 1])

    S = np.array([[1/s_u, 0, 0], 
                  [0, 1/s_v, 0], 
                  [0,  0,  1]])
    C = np.array([[1, 0, -c_u], 
                  [0, 1, -c_v], 
                  [0, 0,   1]])

    # Get T
    T = S @ C

    # Make points into homenous points and take transpose
    points_homogenous = np.ones((3, len(points)))
    points_homogenous[:2] = points.T

    # Get normalized points
    points_normalized = (T @ points_homogenous)[:2].T

    return (points_normalized, T)


def unnormalize_F(F_norm, T_a, T_b):
    """
    Adjusts F to account for normalized coordinates by using the transformation
    matrices.

    Args:
        F_norm: A numpy array of shape (3, 3) representing the normalized
            fundamental matrix
        T_a: Transformation matrix for image A
        T_B: Transformation matrix for image B

    Returns:
        F_orig: A numpy array of shape (3, 3) representing the original
            fundamental matrix
    """

    F_orig = (T_b.T @ F_norm) @ T_a

    return F_orig


def estimate_fundamental_matrix(points_a, points_b):
    """
    Calculates the fundamental matrix. Use the normalize_points() and
    unnormalize_F() functions.

    Args:
        points_a: A numpy array of shape (N, 2) representing the 2D points in
            image A
        points_b: A numpy array of shape (N, 2) representing the 2D points in
            image B

    Returns:
        F: A numpy array of shape (3, 3) representing the fundamental matrix
    """

    A = np.zeros((len(points_a), 9))
        
    # Normalize coordinates
    points_normalized_a, T_a = normalize_points(points_a)
    points_normalized_b, T_b = normalize_points(points_b)

    # Fill in rows of A.
    for i in range(len(points_normalized_a)):
        u_a = points_normalized_a[i][0]
        v_a = points_normalized_a[i][1]
        
        u_b = points_normalized_b[i][0]
        v_b = points_normalized_b[i][1]

        eqn = [u_a*u_b, v_a*u_b, u_b, u_a*v_b, v_a*v_b, v_b, u_a, v_a, 1]

        A[i] = eqn

    # Get eigenvector corresponding to smallest eigenvalue
    eigenvalues, eigenvectors = np.linalg.eig(A.T @ A)
    min_eig_ind = np.argmin(eigenvalues)
    F = np.reshape(eigenvectors[:, min_eig_ind], (3, 3))
    
    # unnormalize
    F = unnormalize_F(F , T_a, T_b)
    
    # enforce rank 2 constraint
    u, s, vh = np.linalg.svd(F)
    s[-1] = 0
    F = (u @ np.diag(s))@vh

    return F
