from radio import Radio
import time

radio = Radio()
print("RX ready - Text mode")

while True:
    if radio.available():
        data = radio.recv()
        
        if not data:
            continue

        # Try to decode as TEXT (since that's what transmitter sends)
        try:
            message = data.decode().strip()
            
            # Print everything - Processing will filter what it needs
            print(message)
            
        except Exception as e:
            # If decode fails, just pass
            pass
    
    time.sleep(0.001)