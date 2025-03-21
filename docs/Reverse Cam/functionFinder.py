import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

csv_file = "data.csv"

# Load data from CSV file
data = np.loadtxt(csv_file, delimiter=";", skiprows=1, usecols=(0, 1))
degrees, lengths = data[:, 0], data[:, 1]

# Test polynomial fits of increasing degrees
max_degree = 20
errors = []

for d in range(1, max_degree + 1):
    coeffs = np.polyfit(degrees, lengths, d)
    poly_func = np.poly1d(coeffs)
    predicted = poly_func(degrees)

    rmse = np.sqrt(mean_squared_error(lengths, predicted))
    errors.append(rmse)

# Find the best polynomial degree
best_degree = np.argmin(errors) + 1
print(f"Best polynomial degree: {best_degree}")

# Get the coefficients of the best polynomial fit
best_coeffs = np.polyfit(degrees, lengths, best_degree)

# Format coefficients for C++ code with 'f' suffix
formatted_coeffs = ", ".join([f"{coeff}f" for coeff in best_coeffs])
print(f"\nC++ vector format:\nconst std::vector<float> CAM_COEFFICIENTS PROGMEM = {{ {formatted_coeffs} }};")

# Generate C++ vector containing data points
print("\nstd::vector<std::pair<float, float>> flight_data = {")
for deg, len in zip(degrees, lengths):
    print(f"    {{{deg}f, {len}f}},")
print("};")

# Plot the data and the best polynomial fit
best_poly_func = np.poly1d(best_coeffs)
x_range = np.linspace(min(degrees), max(degrees), 500)  # Smooth range for plotting
plt.scatter(degrees, lengths, label="Data", color="blue")
plt.plot(x_range, best_poly_func(x_range), label=f"Best Fit (Degree {best_degree})", color="red")
plt.xlabel("Wand Degrees")
plt.ylabel("Vertical Length")
plt.legend()
plt.title("Polynomial Fit to Data")
plt.show()
