import numpy as np

def generate_lambda(n, alpha, kappa):
    '''
    Generates lamba parameter for UT and sigma generation
    '''
    return alpha**2 * (n + kappa) - n

def generate_weights(n, alpha, beta, lambda_val):
    '''
    Generates the mean and covariance weights based on the given parameters
    '''
    wm_0 = lambda_val / (n + lambda_val)
    wc_0 = (lambda_val / (n + lambda_val)) + (1 - alpha**2 + beta)
    wm_i = 0.5 * (1 / (n + lambda_val))
    Wm = np.append(np.array([wm_0]), np.array([wm_i]*(2*n)))
    Wc = np.append(np.array([wc_0]), np.array([wm_i]*(2*n)))
    return Wm, Wc

def generateSigmaPoints(state, covariance, lambda_val):
    '''
    Generate sample points from a state's distribution
    '''
    sigma_pts = []
    covariance_sqrt = np.linalg.cholesky(covariance)
    sigma_pts.append(state)
    for column in covariance_sqrt.T:
        result = np.sqrt(covariance.shape[0] + lambda_val) * column
        result_state = result
        sigma_pts.append(state + result_state)
        sigma_pts.append(state - result_state)
    return sigma_pts

def unscented_transform(sigmas, Wm, Wc):
    '''
    This function performs the unscented transform on a set of sigma points. It computes the weighted mean and covariance of the given sigma points.
    This is a simplified version of the FilterPy implementation.
    Source: https://github.com/rlabbe/filterpy/blob/master/filterpy/kalman/unscented_transform.py
    '''
    x = np.dot(Wm, sigmas)
    y = sigmas - x
    P = np.dot(y.T, np.dot(np.diag(Wc), y))
    return (x, P)



# Testing Unscented Transform Python Implementation
# These value will be used to compare to the C++ implementation
# Refer to test_unscented_transform.cpp for the C++ Unit Test
time_step = 1
state = np.array([5.7441, 1.3800, 2.2049, 0.5015, 0.3528]) # px, py, v, yaw, yaw_rate
covariance = np.array([
    [0.0043, -0.0013, 0.0030, -0.0022, -0.0020],
    [-0.0013, 0.0077, 0.0011,  0.0071,  0.0060],
    [0.0030,  0.0011, 0.0054,  0.0007,  0.0008],
    [-0.0022, 0.0071, 0.0007,  0.0098,  0.0100],
    [-0.0020, 0.0060, 0.0008,  0.0100,  0.0123]
])

# Declaring the UT parameters
n = state.shape[0]
alpha = 1.0
kappa = 1.0
beta = 2.0
lambda_val = generate_lambda(n=n, alpha=alpha, kappa=kappa)

# Generate Weights
Wm, Wc = generate_weights(n, alpha, beta, lambda_val)

# Generate sigma points
sigma_points = generateSigmaPoints(state=state, covariance=covariance, lambda_val=lambda_val)

# Advance mean and sigma points through the non-linear model
predicted_sigma_points = np.array([
                        [7.45259,  2.75566,  2.2049,   0.8543,   0.3528],
                        [7.03431,  2.91475,  2.09284,  1.01119,  0.427509],
                        [7.86347,  2.55502,  2.31696,  0.697412, 0.278091],
                        [7.49311,  2.90276,  2.33349,  0.909401, 0.38609],
                        [7.09247,  3.09087,  2.2049,   1.17264,  0.538586],
                        [7.70496,  2.03736,  2.14739,  0.5153,   0.198194],
                        [7.40311,  2.61378,  2.07631,  0.799199, 0.31951],
                        [7.51905,  2.67765,  2.2049,   0.761074, 0.259574],
                        [7.0649 ,  3.39634,  2.26241,  1.1933,   0.507406],
                        [7.72483,  2.34281,  2.2049,   0.535964, 0.167014],
                        [7.38153,  2.82926,  2.2049,   0.947526, 0.446026]])

result_state, result_covariance = unscented_transform(sigmas=predicted_sigma_points, Wm=Wm, Wc=Wc)

print(f"Wm:\n {Wm}\n")
print(f"Wc:\n {Wc}\n")

print(f"\nGenerated Sigma Points:\n")
for point in sigma_points:
    print(point)

print(f"\nPredicted Sigma Points:\n")
for point in predicted_sigma_points:
    print(point)

print(f"\nState:\n{result_state}")
print(f"\nCovariance:\n{result_covariance}\n")
