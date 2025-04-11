import serial
import csv
import time

# === CONFIGURATION ===
PORT = 'COM22'        # Change this to your Arduino's COM port (e.g., /dev/ttyUSB0 on Linux/Mac)
BAUD_RATE = 115200
CSV_FILE = 'imu_angles.csv'
DURATION = 10        # seconds to record

# === CONNECT TO ARDUINO ===
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Give time for Arduino reset

# === OPEN CSV FILE ===
with open(CSV_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Write CSV header
    writer.writerow([
        'pitch_accel', 'roll_accel',
        'pitch_gyro', 'roll_gyro', 'yaw_gyro',
        'pitch_filter', 'roll_filter'
    ])

    print("Recording IMU data for", DURATION, "seconds...")
    start_time = time.time()

    while time.time() - start_time < DURATION:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 7:
                    writer.writerow(parts)
                    print(parts)
        except Exception as e:
            print("Error:", e)

print("✅ Data saved to", CSV_FILE)
ser.close()



import pandas as pd
import matplotlib.pyplot as plt

# === Load Data ===
df = pd.read_csv('imu_angles.csv')

# === Create Time Axis ===
df['time'] = [i * 0.05 for i in range(len(df))]  # assuming 50ms (20Hz) loop time

# === Plot Pitch Step Response ===
plt.figure(figsize=(10, 6))
plt.plot(df['time'], df['pitch_accel'], label='Accel Pitch', linestyle='--')
plt.plot(df['time'], df['pitch_gyro'], label='Gyro Pitch', linestyle=':')
plt.plot(df['time'], df['pitch_filter'], label='Filtered Pitch', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (°)')
plt.title('Pitch Step Response')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# === Plot Roll Step Response ===
plt.figure(figsize=(10, 6))
plt.plot(df['time'], df['roll_accel'], label='Accel Roll', linestyle='--')
plt.plot(df['time'], df['roll_gyro'], label='Gyro Roll', linestyle=':')
plt.plot(df['time'], df['roll_filter'], label='Filtered Roll', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Roll Angle (°)')
plt.title('Roll Step Response')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

