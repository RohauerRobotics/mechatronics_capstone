import serial
import time

# Limit the amount of time this function can run
start = time.time()
current = time.time()
elapsed_time = 0

pico = serial.Serial(port='COM9',baudrate=115200, timeout=.1)

while elapsed_time < 50:
	current = time.time()
	elapsed_time = current - start
	dat = pico.readline()
	dat = dat.decode('utf-8').strip()
	if len(dat) > 1:
		print(dat)

# pico.write(b'a')

print("While Loop Ended")
