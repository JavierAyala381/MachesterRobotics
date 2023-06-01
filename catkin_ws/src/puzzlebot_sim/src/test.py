import numpy as np

def resize_covariance(covariance_matrix):
    # Initialize a 6x6 matrix filled with zeros
    new_covariance = np.zeros((6, 6))

    # Copy the elements of the 3x3 covariance matrix
    new_covariance[:2, :2] = covariance_matrix[:2, :2]  # Covariances for x and y
    new_covariance[5, :2] = covariance_matrix[2,:2] # Covariance for thetha with respect to x and y
    new_covariance[:2, 5] = covariance_matrix[:2,2]
    new_covariance[5, 5] = covariance_matrix[2, 2]  # Variance for rotation in z

    # Set the covariances involving z and rotation in x, rotation in y, and rotation in z to zero
    new_covariance[2:5, 2:5] = 0.0

    # Return the resulting 6x6 matrix
    return new_covariance

# Example 3x3 covariance matrix
covariance_matrix = np.array([
    [0.1, 0.2, 7.7],
    [0.2, 0.4, 3.5],
    [0.25, 4.8, 0.3]
])

# Resize to a 6x6 matrix with zeros in the missing covariances
new_covariance_matrix = resize_covariance(covariance_matrix)

print(new_covariance_matrix)