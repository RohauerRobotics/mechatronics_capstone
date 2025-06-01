import serial
import time
import matplotlib.pyplot as plt
import pickle

# save data to csv file
filename = "pid_kp=2.25.pkl"

# Limit the amount of time this function can run

pico = serial.Serial(port='COM9',baudrate=115200, timeout=0.1)

buff_len = 1200
sub_buff = 10
count = 0

time_buffer = []
angle_buffer = []
velocity_buffer = []
signal_buffer = []

while count < int(buff_len/sub_buff):
	# collect time
	pico.write(b't')
	time_dat = pico.readline()
	time_dat = time_dat.decode('utf-8').strip()
	time_dat = time_dat.split(',')
	# print(time_dat)
	if len(time_dat) > 1:
		# print(time_dat)
		for i in range(0,len(time_dat)):
			# print(time_dat[i])
			# print(type(time_dat[i]))
			time_buffer.append(float(time_dat[i]))
		# collect angle
		pico.write(b'a')
		angle_dat = pico.readline()
		angle_dat = angle_dat.decode('utf-8').strip()
		angle_dat = angle_dat.split(',')
		if len(angle_dat) > 1:
			for i in range(0,len(angle_dat)):
				angle_buffer.append(float(angle_dat[i]))
			# collect velocity
			pico.write(b'v')
			velocity_dat = pico.readline()
			velocity_dat = velocity_dat.decode('utf-8').strip()
			velocity_dat = velocity_dat.split(',')
			if len(velocity_dat) > 1:
				for i in range(0,len(velocity_dat)):
					velocity_buffer.append(float(velocity_dat[i]))
				pico.write(b's')
				signal_dat = pico.readline()
				signal_dat = signal_dat.decode('utf-8').strip()
				signal_dat = signal_dat.split(',')
				if len(signal_dat) > 1:
					for i in range(0,len(signal_dat)):
						signal_buffer.append(float(signal_dat[i]))
					count += 1
					if (count % 10 == 0):
						print("Collected 100 Data Points")

pico.write(b'z')

# Save the buffers to a file
with open(filename, "wb") as file:
    pickle.dump((time_buffer, angle_buffer, velocity_buffer, signal_buffer), file)

print("Data saved successfully to " , filename)

fig, axes = plt.subplots(3)
fig.suptitle('Angular Displacement and Velocity')
axes[0].scatter(time_buffer, angle_buffer)
axes[0].set_ylabel("Angular Error (Degrees)")
axes[1].scatter(time_buffer, signal_buffer)
axes[1].set_ylabel("Signal Over Time")
axes[2].scatter(time_buffer,velocity_buffer)
axes[2].set_ylabel("Angular Velocity (Degrees/Seconds)")
plt.xlabel("Time(s)")
plt.tight_layout()
plt.show()


print("While Loop Ended")
