import json
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats

# Load JSON data from the file
with open('short_data.json', 'r') as file:
    data = json.load(file)

# Extract yaw, pitch, roll, and timestamps
yaws = [entry['yaw'] for entry in data]
pitches = [entry['pitch'] for entry in data]
rolls = [entry['roll'] for entry in data]
timestamps = [entry['timestamp'] for entry in data]
timestamps= np.array([x-timestamps[0] for x in timestamps])

# Extract magnetometer, accelerometer, and gyroscope values
mag_x = [entry['mag_x'] for entry in data]
mag_y = [entry['mag_y'] for entry in data]
mag_z = [entry['mag_z'] for entry in data]

accel_x = [entry['accel_x'] for entry in data]
accel_y = [entry['accel_y'] for entry in data]
accel_z = [entry['accel_z'] for entry in data]

gyro_x = [entry['gyro_x'] for entry in data]
gyro_y = [entry['gyro_y'] for entry in data]
gyro_z = [entry['gyro_z'] for entry in data]

# # Function to calculate mean, standard deviation, and variance
# def calc_stats(values):
#     mean_val = np.mean(values)
#     std_val = np.std(values)
#     var_val = np.var(values)
#     return mean_val, std_val, var_val

# # Function to plot data with statistics
# def plot_data(timestamps, values, title, ylabel, color):
#     mean_val, std_val, var_val = calc_stats(values)
    
#     plt.figure(figsize=(15, 4))
#     plt.plot(timestamps, values, color=color, linewidth=0.25, alpha=0.8)  # Removed the marker
#     plt.title(f'{title} Over Time\nMean: {mean_val:.5f}, SD: {std_val:.5f}, Variance: {var_val:.5f}')
#     plt.xlabel('Time (sec)')
#     plt.ylabel(ylabel)
#     plt.grid()
#     plt.tight_layout()
#     plt.show()

# # Plot Yaw, Pitch, and Roll
# plot_data(timestamps, yaws, 'Yaw', 'Yaw (degrees)', 'blue')
# plot_data(timestamps, pitches, 'Pitch', 'Pitch (degrees)', 'green')
# plot_data(timestamps, rolls, 'Roll', 'Roll (degrees)', 'red')

# # Plot Magnetometer X (blue), Y (green), Z (red)
# plot_data(timestamps, mag_x, 'Magnetometer X', 'Magnetometer X (Tesla)', 'blue')
# plot_data(timestamps, mag_y, 'Magnetometer Y', 'Magnetometer Y (Tesla)', 'green')
# plot_data(timestamps, mag_z, 'Magnetometer Z', 'Magnetometer Z (Tesla)', 'red')

# # Plot Accelerometer X (blue), Y (green), Z (red)
# plot_data(timestamps, accel_x, 'Accelerometer X', 'Accel X (m/s²)', 'blue')
# plot_data(timestamps, accel_y, 'Accelerometer Y', 'Accel Y (m/s²)', 'green')
# plot_data(timestamps, accel_z, 'Accelerometer Z', 'Accel Z (m/s²)', 'red')

# # Plot Gyroscope X (blue), Y (green), Z (red)
# plot_data(timestamps, gyro_x, 'Gyroscope X', 'Gyro X (rad/s)', 'blue')
# plot_data(timestamps, gyro_y, 'Gyroscope Y', 'Gyro Y (rad/s)', 'green')
# plot_data(timestamps, gyro_z, 'Gyroscope Z', 'Gyro Z (rad/s)', 'red')

def calc_error(values):
    mean_val = np.mean(values)
    return [value - mean_val for value in values], mean_val

# Function to plot histogram and fitted normal distribution
def plot_error_distribution(errors, title, color, bins):
    # Fit a normal distribution to the data
    mu, std = stats.norm.fit(errors)

    # Plot the histogram of the error distribution
    plt.figure(figsize=(8, 4))
    counts, bin_edges, _ = plt.hist(errors, bins=bins, density=True, color=color, alpha=0.6, label='Error Distribution')

    # Plot the fitted normal distribution curve
    bin_centers = 0.5 * (bin_edges[1:] + bin_edges[:-1])
    plt.plot(bin_centers, stats.norm.pdf(bin_centers, mu, std), color='black', linestyle='dashed', linewidth=2, label='Fitted Normal Curve')

    plt.title(f'{title} Error Distribution')
    plt.xlabel('Error')
    plt.ylabel('Density')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

# Datasets to evaluate with respective bin counts
datasets = [
    ('Yaw', yaws, 'blue', 100),  # 100 bins for Yaw
    ('Pitch', pitches, 'green', 100),  # 100 bins for Pitch
    ('Roll', rolls, 'red', 100),  # 100 bins for Roll
    ('Magnetometer X', mag_x, 'blue', 25),  # 15 bins for Magnetometer
    ('Magnetometer Y', mag_y, 'green', 25),  # 15 bins for Magnetometer
    ('Magnetometer Z', mag_z, 'red', 25),  # 15 bins for Magnetometer
    ('Accelerometer X', accel_x, 'blue', 30),  # 50 bins for Accelerometer
    ('Accelerometer Y', accel_y, 'green', 30),  # 50 bins for Accelerometer
    ('Accelerometer Z', accel_z, 'red', 30),  # 50 bins for Accelerometer
    ('Gyroscope X', gyro_x, 'blue', 100),  # 100 bins for Gyroscope
    ('Gyroscope Y', gyro_y, 'green', 100),  # 100 bins for Gyroscope
    ('Gyroscope Z', gyro_z, 'red', 100)   # 100 bins for Gyroscope
]

# Plot for Yaw, Pitch, Roll with fitted normal curve
for title, values, color, bins in datasets[:3]:  # Only for yaw, pitch, roll
    errors, mean_val = calc_error(values)
    plot_error_distribution(errors, title, color, bins)

# Plot with normal fit for Magnetometer, Accelerometer, and Gyroscope
for title, values, color, bins in datasets[3:]:
    errors, mean_val = calc_error(values)
    plot_error_distribution(errors, title, color, bins)





