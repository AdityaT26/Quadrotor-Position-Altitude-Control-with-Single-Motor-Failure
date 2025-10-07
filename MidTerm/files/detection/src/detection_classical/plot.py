import csv
import matplotlib.pyplot as plt
import datetime
import numpy as np

# Function to apply a moving average filter
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode="valid")

# Function to plot data from the CSV file
def plot_data1(filename1, window_size=5):
    timestamps, series1, series2, series3 = [], [], [], []

    with open(filename1, "r") as file:
        reader = csv.reader(file)
        for row in reader:
            if row:
                # Convert nanoseconds to seconds and then to datetime
                nanosecond_timestamp = int(row[0])
                timestamp = datetime.datetime.fromtimestamp(nanosecond_timestamp / 1e9)
                timestamps.append(timestamp)

                # Convert values to float and append
                series1.append(float(row[1]))
                series2.append(float(row[2]))
                series3.append(float(row[3]))

    # Apply moving average filter to each series
    series1_smoothed = moving_average(series1, window_size)
    series2_smoothed = moving_average(series2, window_size)
    series3_smoothed = moving_average(series3, window_size)
    
    # Adjust timestamps to match filtered data length
    timestamps_smoothed = timestamps[:len(series1_smoothed)]

    plt.figure(figsize=(10, 6))
    plt.plot(timestamps_smoothed, series1_smoothed, label="R1", color="blue")
    plt.plot(timestamps_smoothed, series2_smoothed, label="R2", color="green")
    plt.plot(timestamps_smoothed, series3_smoothed, label="R3", color="red")
    plt.xlabel("Time")
    plt.ylabel("Values")
    plt.title("Residuals")
    plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()

filename1 = "./timeseries_residuals.csv"
# filename2 = "./timeseries_torque_setpoint.csv"
# Plot data from CSV with a moving average window size of 5
plot_data1(filename1, window_size=10)
# plot_data2(filename2, window_size=10)
