from radio import Radio
import time

radio = Radio()
counter = 0

print("Starting radio test...")

while True:
    msg = "AVO {}".format(counter)
    print("Sending:", msg)

    radio.send(msg.encode())

    counter += 1
    time.sleep(0.5)