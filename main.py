from radio import Radio
import time

radio = Radio()

print("RX ready")

while True:
    if radio.available():
        data = radio.recv()
        if data:
            try:
                msg = data.decode()
                parts = msg.split("|")

                if parts[0] == "AVO":
                    counter = parts[1]
                    print("Packet OK → Counter:", counter)

                else:
                    print("Unknown packet:", msg)

            except Exception as e:
                print("Decode error:", e)

    time.sleep(0.05)