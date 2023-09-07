import numpy as np
from src.estimate_fundamental_matrix import estimate_fundamental_matrix


def calculate_num_ransac_iterations(
    prob_success: float, sample_size: int, ind_prob_correct: float
) -> int:
    """
    Calculates the number of RANSAC iterations needed for a given guarantee of
    success.

    Args:
        prob_success: float representing the desired guarantee of success
        sample_size: int the number of samples included in each RANSAC
            iteration
        ind_prob_success: float representing the probability that each element
            in a sample is correct

    Returns:
        num_samples: int the number of RANSAC iterations needed

    """
    p = prob_success
    e = 1-ind_prob_correct
    s = sample_size

    num_samples = np.log(1-p)/(np.log(1-(1-e)**s))

    return int(num_samples)


def ransac_fundamental_matrix(
    matches_a: np.ndarray, matches_b: np.ndarray
) -> np.ndarray:
    """
    Use RANSAC to find the best fundamental matrix by
    randomly sampling interest points. 

    Args:
        matches_a: A numpy array of shape (N, 2) representing the coordinates
            of possibly matching points from image A
        matches_b: A numpy array of shape (N, 2) representing the coordinates
            of possibly matching points from image B
    Each row is a correspondence (e.g. row 42 of matches_a is a point that
    corresponds to row 42 of matches_b)

    Returns:
        best_F: A numpy array of shape (3, 3) representing the best fundamental
            matrix estimation
        inliers_a: A numpy array of shape (M, 2) representing the subset of
            corresponding points from image A that are inliers with respect to
            best_F
        inliers_b: A numpy array of shape (M, 2) representing the subset of
            corresponding points from image B that are inliers with respect to
            best_F
    """

    prob_success=0.9999
    sample_size = 9
    ind_prob_correct=0.5
    threshold = 0.1

    # Get all indicies to choose smaples from.
    all_indicies = np.arange(len(matches_a))
    
    # Record best F and num inliers
    best_F = np.zeros((3, 3))
    best_inlier_indicies = []
    best_inlier_scores = []

    # Get numiters necessary
    min_iters = calculate_num_ransac_iterations(prob_success,sample_size, ind_prob_correct)

    k = 0
    while k < min_iters:
        #### ESTIMATE F ####
        ####################
        # Sample indicies
        sample_inds = np.random.choice(all_indicies, sample_size, replace=False)
        # Get points corresponding to sampled indicies
        points_a = matches_a[sample_inds]
        points_b = matches_b[sample_inds]
        # Get F
        F = estimate_fundamental_matrix(points_a, points_b)
        ####################
        ####################

        inlier_indicies = []
        inlier_scores = []
        ### GET INLIERS FOR F ###
        #########################
        # Calculate xF before xFx^.T
        matches_a_with_1 = np.ones((len(matches_a), 3))
        matches_a_with_1[:, :2] = matches_a
        rows = matches_a_with_1 @ F 

        for i in range(len(matches_a)):
            score = rows[i] @ np.append(matches_b[i], 1).T
            if score < threshold:
                inlier_indicies.append(i)
                inlier_scores.append(score)
                
        # If better than current best, record everything.
        if len(inlier_indicies) > len(best_inlier_indicies):
            # Update.
            best_inlier_indicies = np.array(inlier_indicies)
            best_inlier_scores = np.array(inlier_scores)
            best_F = F

        k += 1

    # Sort indicies of matches by the score on best F.
    top_scores_ind = np.argsort(best_inlier_scores)
    top_inds = best_inlier_indicies[top_scores_ind]

    inliers_a = matches_a[top_inds]
    inliers_b = matches_b[top_inds]

    return best_F, inliers_a, inliers_b
