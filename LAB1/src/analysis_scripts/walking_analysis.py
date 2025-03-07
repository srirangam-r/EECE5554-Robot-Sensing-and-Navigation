import json
import matplotlib.pyplot as plt
import utm
import math
import gmplot
import numpy as np
import scipy.stats as stats
from scipy.special import gamma

import seaborn as sns
json_file_path = 'walking_data.json'

with open(json_file_path, 'r') as file:
    data = json.load(file)

start_pos = [42.33655903142405, -71.08778605288846]
end_pos = [42.3390555566669, -71.08380574279496]

start_utm = utm.from_latlon(start_pos[0],start_pos[1])
end_utm = utm.from_latlon(end_pos[0],end_pos[1])

x1 = start_utm[0]
y1 = start_utm[1]
x2 = end_utm[0]
y2 = end_utm[1]


timestamps = [entry["timestamp"] for entry in data]
latitudes = [entry["latitude"] for entry in data]
longitudes = [entry["longitude"] for entry in data]
altitudes = [entry["altitude"] for entry in data]
utm_easting = np.array([entry["utm_easting"] for entry in data])
utm_northing = np.array([entry["utm_northing"] for entry in data])

A = y2 - y1
B = x1 - x2
C = x2 * y1 - x1 * y2
print(A,B,C)

# Function to compute perpendicular distance from the line
def perpendicular_distance(x, y, A, B, C):
    return (np.abs(A * x + B * y + C) / np.sqrt(A**2 + B**2))*100

errors = perpendicular_distance(utm_easting, utm_northing, A, B, C)

utm_easting= np.array([x-x1 for x in utm_easting])
utm_northing= np.array([x-y1 for x in utm_northing])
timestamps= np.array([x-timestamps[0] for x in timestamps])
positions = np.column_stack((utm_easting, utm_northing))

errors = errors[160::1]
timestamps = timestamps[160::1]


# Ensure errors is a NumPy array
errors = np.asarray(errors)

# Fit a Log-Normal distribution to the data
shape, loc, scale = stats.lognorm.fit(errors, floc=0)  # floc=0 to fix location at 0

# Generate points for plotting the fitted Log-Normal distribution
x = np.linspace(min(errors), max(errors), 1000)
pdf_fitted = stats.lognorm.pdf(x, shape, loc, scale)

# Plot the histogram and the fitted Log-Normal distribution
plt.figure(figsize=(10, 6))
sns.histplot(errors, bins=30, stat="density", kde=False, label='Data Histogram', color='lightblue')
plt.plot(x, pdf_fitted, 'r-', label='Fitted Log-Normal Distribution', linewidth=2)
plt.title('Log-Normal Fit to Error Data')
plt.xlabel('Errors')
plt.ylabel('Density')
plt.legend()
plt.show()

# Perform Kolmogorov-Smirnov (KS) Test
# Define the CDF of the fitted Log-Normal distribution
cdf_fitted = lambda y: stats.lognorm.cdf(y, shape, loc, scale)

# Perform KS test comparing the data with the fitted distribution
ks_stat, ks_pvalue = stats.kstest(errors, cdf_fitted)

# Output the results
print(f'Fitted Log-Normal Distribution Parameters:\nShape: {shape}, Location: {loc}, Scale: {scale}')
print(f'KS Test Statistic: {ks_stat}')
print(f'KS Test p-value: {ks_pvalue}')


# print(np.mean(errors))
# params = stats.weibull_min.fit(errors, floc=0)  # floc=0 fixes the location to 0
# shape, loc, scale = params

# # Generate points for plotting the fitted Weibull distribution
# x = np.linspace(min(errors), max(errors), 1000)
# pdf_fitted = stats.weibull_min.pdf(x, shape, loc, scale)

# # Plot the histogram and the fitted Weibull distribution
# plt.figure(figsize=(10, 6))
# sns.histplot(errors, bins=30, stat="density", kde=False, label='Data Histogram', color='lightblue')
# plt.plot(x, pdf_fitted, 'r-', label='Fitted Weibull Distribution', linewidth=2)
# plt.title('Weibull Fit to Error Data')
# plt.xlabel('Errors')
# plt.ylabel('Density')
# plt.legend()
# plt.show()

# # Print the parameters of the fitted Weibull distribution
# print(f'Fitted Weibull Distribution Parameters:\nShape: {shape}, Location: {loc}, Scale: {scale}')

# # Goodness-of-fit using the Kolmogorov-Smirnov test
# ks_statistic, ks_p_value = stats.kstest(errors, 'weibull_min', args=params)
# print(f'Kolmogorov-Smirnov Test: Statistic={ks_statistic}, p-value={ks_p_value}')

# mean_error = scale * gamma(1 + 1/shape)

# # RMSE (Root Mean Square Error)
# rmse_error = scale * gamma(1 + 2/shape)

# # Median Error (P50)
# median_error = stats.weibull_min.median(shape, loc=0, scale=scale)

# # P95 Error
# p95_error = stats.weibull_min.ppf(0.90, shape, loc=0, scale=scale)

# print(f"Mean Error: {mean_error} cm")
# print(f"RMSE: {rmse_error} cm")
# print(f"Median Error: {median_error} cm")
# print(f"95th Percentile Error: {p95_error} cm")

# a, b = np.polyfit(utm_easting,utm_northing, 1)
# print(a,b)

# plt.figure(figsize=(10, 6))
# plt.plot([0, x2-x1], [0, y2-y1], 'r-', label="True path")

# # Plotting the curve containing all pairs from the UTM coordinates
# plt.plot(utm_easting, utm_northing, 'b-', label="GPS data")
# plt.plot(utm_easting, a*utm_easting+b, 'g-', label="Fitted line", linestyle='dashed')

# # Adding labels and title
# plt.xlabel('UTM Easting(m)')
# plt.ylabel('UTM Northing(m)')

# # Add legend to distinguish between the line and the curve
# plt.legend()

# plt.show()

# Plotting the distance vs timestamp
plt.figure(figsize=(10, 6))
plt.plot(timestamps, errors, color='tab:blue', label='error')
plt.xlabel('time (sec)')
plt.ylabel('error (cm)')
plt.title('time')
plt.grid(True)
plt.legend()

# Show the plot
plt.show()

print(min(errors),max(errors),np.mean(errors), np.std(errors))
rmse = np.sqrt(np.mean(errors**2))
print(rmse)


# fig, ax1 = plt.subplots()
# # Show the plot
# plt.show()

# ax1.set_xlabel('Timestamp')
# ax1.set_ylabel('Latitude_error', color='tab:blue')
# ax1.plot(timestamps, lat_error, color='tab:blue', label='Latitude_error')
# ax1.tick_params(axis='y', labelcolor='tab:blue')

# ax2 = ax1.twinx()
# ax2.set_ylabel('Longitude_error', color='tab:green')
# ax2.plot(timestamps, lon_error, color='tab:green', label='Longitude_error')
# ax2.tick_params(axis='y', labelcolor='tab:green')

# # ax3 = ax1.twinx()
# # ax3.spines['right'].set_position(('outward', 120))  
# # ax3.set_ylabel('Altitude', color='tab:red')
# # ax3.plot(timestamps, altitudes, color='tab:red', label='Altitude')
# # ax3.tick_params(axis='y', labelcolor='tab:red')


# plt.title('Latitude and longitude error vs Timestamp')
# fig.tight_layout()
# plt.grid()


# plt.show()
