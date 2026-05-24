from radio import Radio
import time

radio = Radio()
print("RX ready")

while True:
    if radio.available():
        data = radio.recv()
        if not data:
            continue

        try:
            msg = data.decode(errors="ignore").strip()
            print("RAW:", msg)

            parts = msg.split("|")

            if parts[0] != "AVO":
                print("Non-AVO packet:", msg)
                continue

            pkt_type = parts[1]

            if pkt_type == "A":
                print("GYRO PACKET:", parts[2:])

            elif pkt_type == "B":
                print("SENSOR PACKET:", parts[2:])

            else:
                print("Unknown AVO type:", msg)

        except Exception as e:
            print("Decode error:", e)

    time.sleep(0.05)