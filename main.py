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
            msg = data.decode().strip()

            parts = msg.split(",")

            if parts[0] == "AVO" and len(parts) >= 8:

                gx = float(parts[1])
                gy = float(parts[2])
                gz = float(parts[3])
                alt = float(parts[4])
                ax  = float(parts[5])
                ay  = float(parts[6])
                az  = float(parts[7])

                print("----- PACKET OK -----")
                print("Gyro:", gx, gy, gz)
                print("Accel:", ax, ay, az)
                print("Alt(ft):", alt)
                print("---------------------")

            else:
                print("Bad packet format:", msg)

        except Exception as e:
            print("Decode error:", e)

    time.sleep(0.05)