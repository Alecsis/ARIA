from radio import Radio
import time

radio = Radio()

print("Waiting for messages...")

while True:
    if radio.available():
        data = radio.recv()
        print("Received:", data)

    time.sleep(0.05)