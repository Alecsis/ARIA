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

            if len(parts) != 9:
                print("Bad packet length:", len(parts))
                continue

            if parts[0] != "AVO":
                print("Non-AVO packet:", msg)
                continue

            counter = int(parts[1])

            gx = float(parts[2])
            gy = float(parts[3])
            gz = float(parts[4])

            alt = float(parts[5])

            ax = float(parts[6])
            ay = float(parts[7])
            az = float(parts[8])

            print("----- PACKET OK -----")
            print("Counter:", counter)
            print("Gyro:", gx, gy, gz)
            print("Accel:", ax, ay, az)
            print("Alt(ft):", alt)
            print("---------------------")

        except Exception as e:
            print("Decode error:", e)

    time.sleep(0.05)