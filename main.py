from radio import Radio
import time
from machine import Pin, Timer
from lib.ws2812 import WS2812  


led_power = Pin(11, Pin.OUT)
led_power.value(1) 

pixel = WS2812(12, 1)

COLOR_RED    = (30, 0, 0)    # Soft Red
COLOR_GREEN  = (0, 30, 0)    # Soft Green
COLOR_BLACK  = (0, 0, 0)     # Off

system_status = "STARTUP"   
current_color = COLOR_BLACK
blink_toggle = False
last_packet_time = time.ticks_ms()

def led_timer_callback(timer):
    global system_status, current_color, blink_toggle, last_packet_time
    
    if system_status == "LINK_OK" and time.ticks_diff(time.ticks_ms(), last_packet_time) > 1000:
        system_status = "LINK_FAIL"

    if system_status == "STARTUP":
        current_color = COLOR_RED
        
    elif system_status == "LINK_OK":
        current_color = COLOR_GREEN
        
    elif system_status == "LINK_FAIL":
        blink_toggle = not blink_toggle
        current_color = COLOR_RED if blink_toggle else COLOR_BLACK

    pixel.pixels_fill(current_color)
    pixel.pixels_show()

led_timer = Timer(-1)
led_timer.init(period=200, mode=Timer.PERIODIC, callback=led_timer_callback)

system_status = "STARTUP"
radio = Radio()
print("RX ready - Text mode")

while True:
    if radio.available():
        data = radio.recv()
        
        if not data:
            continue

        try:
            message = data.decode().strip()
            print(message)
            last_packet_time = time.ticks_ms()
            system_status = "LINK_OK"
            
        except Exception as e:
            pass
    
    time.sleep(0.001)