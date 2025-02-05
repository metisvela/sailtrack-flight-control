import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

csv_file = "data.csv" 

data = np.loadtxt(csv_file, delimiter=";", skiprows=1, usecols=(0, 1))
degrees, distances = data[:, 0], data[:, 1]

# Try polynomial fits of increasing degree
max_degree = 20
errors = []

for d in range(1, max_degree + 1):
    coeffs = np.polyfit(degrees, distances, d)
    poly_func = np.poly1d(coeffs)
    predicted = poly_func(degrees)
    
    rmse = np.sqrt(mean_squared_error(distances, predicted))
    errors.append(rmse)

# Find the best polynomial degree
best_degree = np.argmin(errors) + 1
print(f"Best polynomial degree: {best_degree}")

# Get the coefficients of the best polynomial fit
best_coeffs = np.polyfit(degrees, distances, best_degree)

# Format coefficients for C++ code with 'f' suffix
formatted_coeffs = ", ".join([f"{coeff}f" for coeff in best_coeffs])
print(f"\nC++ vector format:\nvector<float> vec = {{ {formatted_coeffs} }};")

# Plot the data and the best polynomial fit
best_poly_func = np.poly1d(best_coeffs)
plt.scatter(degrees, distances, label="Data", color="blue")
plt.plot(degrees, best_poly_func(degrees), label=f"Best Fit (Degree {best_degree})", color="red")
plt.xlabel("Degrees")
plt.ylabel("Distances")
plt.legend()
plt.title("Polynomial Fit to Data")
plt.show()
