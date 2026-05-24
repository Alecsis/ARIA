from radio import Radio
import time

radio = Radio()

print("RX ready")

while True:
    if radio.available():
        data = radio.recv()
        if data:
            try:
                print("Received:", data.decode())
            except:
                print("Received raw:", data)

    time.sleep(0.05)