import serial
import matplotlib.pyplot as plt

# Connect to your Arduino port (adjust 'COMx' or '/dev/ttyUSBx' as needed)
ser = serial.Serial('COM11', 115200, timeout=1)

# Store data
pitch_accel_list, pitch_gyro_list, pitch_filter_list = [], [], []
roll_accel_list, roll_gyro_list, roll_filter_list = [], [], []

# Number of data points to collect
N = 500

print("Collecting data...")

while len(pitch_accel_list) < N:
    line = ser.readline().decode('utf-8').strip()
    try:
        values = list(map(float, line.split(',')))
        if len(values) == 7:
            pitch_accel, roll_accel = values[0], values[1]
            pitch_gyro, roll_gyro = values[2], values[3]
            pitch_filter, roll_filter = values[5], values[6]

            pitch_accel_list.append(pitch_accel)
            pitch_gyro_list.append(pitch_gyro)
            pitch_filter_list.append(pitch_filter)

            roll_accel_list.append(roll_accel)
            roll_gyro_list.append(roll_gyro)
            roll_filter_list.append(roll_filter)

    except ValueError:
        continue  # skip invalid lines

ser.close()
print("Done collecting!")

# === Plot PITCH ===
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(pitch_accel_list, label='Pitch Accel', color='r')
plt.plot(pitch_gyro_list, label='Pitch Gyro', color='g')
plt.plot(pitch_filter_list, label='Pitch Filter', color='b')
plt.title("Pitch Angles")
plt.xlabel("Sample")
plt.ylabel("Degrees")
plt.legend()
plt.grid(True)

# === Plot ROLL ===
plt.subplot(2, 1, 2)
plt.plot(roll_accel_list, label='Roll Accel', color='r')
plt.plot(roll_gyro_list, label='Roll Gyro', color='g')
plt.plot(roll_filter_list, label='Roll Filter', color='b')
plt.title("Roll Angles")
plt.xlabel("Sample")
plt.ylabel("Degrees")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
