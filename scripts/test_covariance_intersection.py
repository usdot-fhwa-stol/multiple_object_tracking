import numpy as np

def covarianceIntersection(x1, P1_inv, x2, P2_inv, w):
    '''
    CI if you prefer looking at concise notation
    if you prefer more readable variables, refer to function below
    '''
    P_inv = w * P1_inv + (1 - w) * P2_inv
    P = np.linalg.inv(P_inv)
    x = P @ (w * P1_inv @ x1 + (1 - w) * P2_inv @ x2)
    return x, P

def computeCovarianceIntersection(mean1, inverse_covariance1, mean2, inverse_covariance2, weight):
    '''
    Compute the covariance intersection of two multivariate Gaussian distributions.
    '''
    inverse_covariance_combined = weight * inverse_covariance1 + (1 - weight) * inverse_covariance2
    covariance_combined = np.linalg.inv(inverse_covariance_combined)
    mean_combined = covariance_combined @ (weight * inverse_covariance1 @ mean1 + (1 - weight) * inverse_covariance2 @ mean2)
    return mean_combined, covariance_combined

def computeWeight(inverse_covariance1, inverse_covariance2):
    '''
    Generate the weight for covariance intersection of two Gaussian distributions.
    '''
    det_inverse_covariance1 = np.linalg.det(inverse_covariance1)
    det_inverse_covariance2 = np.linalg.det(inverse_covariance2)
    det_inverse_covariance_sum = np.linalg.det(inverse_covariance1 + inverse_covariance2)

    weight = (det_inverse_covariance_sum - det_inverse_covariance2 + det_inverse_covariance1) / (2 * det_inverse_covariance_sum)
    return weight

# Test 1: Fusion example of size 3 vector
x1 = np.array([[1], [2], [3]])  # Mean value 1
P1 = np.array([[4, 0, 0], [0, 5, 0], [0, 0, 6]])  # covariance matrix 1

x2 = np.array([[4], [5], [6]])  # Mean value 2
P2 = np.array([[7, 0, 0], [0, 8, 0], [0, 0, 9]])  # covariance matrix 2

P1_inv = np.linalg.inv(P1)
P2_inv = np.linalg.inv(P2)

w = computeWeight(P1_inv, P2_inv)
x, P = computeCovarianceIntersection(x1, P1_inv, x2, P2_inv, w)
print("Test 1: \n")
print("Weight:")
print(w)
print("\nFused mean value (x):")
print(x)
print("Fused covariance matrix (P):")
print(P)
print("\n---------------------------------------------------------\n")


# Test 2: CTRV Track and detection fusion example
track_state = np.array([[1], [2], [3] , [4], [5]])  # px, py, v, yaw, yaw_rate
track_covariance = np.array([[-0.71124, 0.25895, 0.07334, 0.90743, -0.90324],
                            [-0.48288, -0.45900, -0.80263, 0.66087, -0.50648],
                            [-0.71805, -0.05412, -0.70058, 0.70838, -0.43610],
                            [-0.64773, 0.71184, 0.73016, -0.34521, 0.86354],
                            [-0.28654, -0.55006, 0.21601, -0.75024, -0.42627]])

detection_state = np.array([[1.16422], [1.65312], [3.00000] , [2.46081], [5.00000]]) # px, py, v, yaw, yaw_rate
detection_covariance = np.array([[1.83854, 0.34738, 0.41240, -2.36064, 0.46715],
                                [0.34738, 1.42875, 0.49256, 1.63664, 0.57979],
                                [0.41240, 0.49256, 1.00934, 0.00066, 0.08419],
                                [-2.36064, 1.63664, 0.00066, 9.05961, -0.54188],
                                [0.46715, 0.57979, 0.08419, -0.54188, 1.17590]])

P1_inv = np.linalg.inv(track_covariance)
P2_inv = np.linalg.inv(detection_covariance)

w = computeWeight(P1_inv, P2_inv)
x, P = computeCovarianceIntersection(track_state, P1_inv, detection_state, P2_inv, w)
print("Test 2: \n")
print("Weight:")
print(w)
print("\nFused mean value (x):")
print(x)
print("Fused covariance matrix (P):")
print(P)
print("\n---------------------------------------------------------\n")

# Test 3: CTRA Track and detection fusion example
track_state = np.array([[1], [2], [3] , [4], [5], [6] ])  # px, py, v, yaw, yaw_rate, acceleration
track_covariance = np.array([[0.66016, 0.67280, 0.87234, -0.04035, 0.81610, 0.40492],
                            [0.12663, 0.28159, -0.50021, 0.44237, -0.75851, 0.96020],
                            [-0.44479, 0.23876, -0.59174, -0.53774, 0.67688, 0.92271],
                            [-0.96353, -0.97986, 0.67010, 0.64093, 0.21063, -0.17695],
                            [-0.69443, 0.34330, 0.70980, -0.98922, 0.32312, 0.44862],
                            [0.31894, -0.46326, 0.41500, -0.07116, -0.66293, -0.81833]])

detection_state = np.array([[1.29581], [ 1.59029], [6.0] , [2.18031], [5.0], [6.0]]) # px, py, v, yaw, yaw_rate, acceleration
detection_covariance = np.array([[3.32452, -0.90824, -0.52113, -3.57288, -0.81460, 0.34692],
                                [-0.90824, 2.57238, -0.17246, 2.27956, 0.13771, -0.69168],
                                [-0.52113, -0.17246, 0.56438, 0.55992, 0.07005, 0.13371],
                                [-3.57288, 2.27956, 0.55992, 8.25950, 1.02821, 0.45153],
                                [-0.81460, 0.13771, 0.07005, 1.02821, 3.19971, -1.15597],
                                [0.34692, -0.69168, 0.13371, 0.45153, -1.15597, 2.50944]])

P1_inv = np.linalg.inv(track_covariance)
P2_inv = np.linalg.inv(detection_covariance)

w = computeWeight(P1_inv, P2_inv)
x, P = computeCovarianceIntersection(track_state, P1_inv, detection_state, P2_inv, w)
print("Test 3: \n")
print("Weight:")
print(w)
print("\nFused mean value (x):")
print(x)
print("Fused covariance matrix (P):")
print(P)
print("\n---------------------------------------------------------\n")
