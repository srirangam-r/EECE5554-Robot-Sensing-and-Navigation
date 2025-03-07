import json
import matplotlib.pyplot as plt
import utm
import math
import gmplot
import numpy as np
import scipy.stats as stats
from scipy.special import gamma
import seaborn as sns


json_file_path = 'stationary_data.json'

with open(json_file_path, 'r') as file:
    data = json.load(file)

correct_lat = 42.33889151954649
correct_lon = -71.08831012662841

correct_utm = utm.from_latlon(correct_lat,correct_lon)


timestamps = [entry["timestamp"] for entry in data]
latitudes = [entry["latitude"] for entry in data]
longitudes = [entry["longitude"] for entry in data]
altitudes = [entry["altitude"] for entry in data]
utm_easting = np.array([entry["utm_easting"] for entry in data])
utm_northing = np.array([entry["utm_northing"] for entry in data])

lat_error = [abs(correct_lat-a) for a in latitudes]
lon_error = [abs(correct_lon-a) for a in longitudes]


errors = np.array([(math.sqrt((easting - correct_utm[0])**2 + (northing - correct_utm[1])**2))*100 for easting, northing in zip(utm_easting, utm_northing)])

# gmap = gmplot.GoogleMapPlotter(correct_lat, correct_lon, 100, 'hybrid')
# gmap.scatter([correct_lat], [correct_lon], '#FF0000', size=0.3, marker=True,face_alpha=1.0)
# gmap.scatter(latitudes, longitudes, '#0000FF', size=0.05, marker=False,face_alpha=1.0)


# gmap.draw("stationary.html")

utm_easting= np.array([x-correct_utm[0] for x in utm_easting])
utm_northing= np.array([x-correct_utm[1] for x in utm_northing])
timestamps= np.array([x-timestamps[0] for x in timestamps])
positions = np.column_stack((utm_easting, utm_northing))

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



# print(correct_utm)

# plt.scatter(utm_easting, utm_northing, color='blue', label='GPS data',s=5)

# # Highlight the correct point
# plt.scatter(0,0, color='red', label='True position', s=100)  # Larger size for emphasis
# plt.scatter(np.mean(utm_easting),np.mean(utm_northing), color='green', label='Mean GPS position', s=100)
# # Add labels and title
# plt.xlabel('UTM Easting (m)')
# plt.ylabel('UTM Northing (m)')
# plt.legend()

# print(np.std(utm_easting))
# print(np.std(utm_northing))
# print(np.std(positions,axis=0))

# # Show plot
# plt.show()

# Plotting the distance vs timestamp
plt.figure(figsize=(10, 6))
plt.plot(timestamps, altitudes, color='tab:blue', label='Positional error')
plt.xlabel('Timestamp')
plt.ylabel('Error (cm)')
plt.grid(True)
plt.legend()

# Show the plot
plt.show()

# # print(min(errors),max(errors),np.mean(errors), np.std(errors))
# rmse = np.sqrt(np.mean(errors**2))
# print(rmse)



# fig, ax1 = plt.subplots()

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
