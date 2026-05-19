from machine import Pin, SPI
from nrf24l01 import NRF24L01
import time


class Radio:
    def __init__(self,
                 spi_id=0,
                 sck=8,
                 mosi=10,
                 miso=9,
                 ce_pin=1,
                 csn_pin=2,
                 channel=46,
                 payload_size=16,
                 pipe_tx=b"\xe1\xf0\xf0\xf0\xf0",
                 pipe_rx=b"\xd2\xf0\xf0\xf0\xf0"):

        # SPI setup
        self.spi = SPI(
            spi_id,
            baudrate=4000000,
            polarity=0,
            phase=0,
            sck=Pin(sck),
            mosi=Pin(mosi),
            miso=Pin(miso)
        )

        self.ce = Pin(ce_pin, Pin.OUT)
        self.csn = Pin(csn_pin, Pin.OUT)

        self.nrf = NRF24L01(self.spi, self.csn, self.ce,
                            channel=channel,
                            payload_size=payload_size)

        self.pipe_tx = pipe_tx
        self.pipe_rx = pipe_rx

        self.nrf.open_tx_pipe(self.pipe_tx)
        self.nrf.open_rx_pipe(1, self.pipe_rx)
        self.nrf.start_listening()

        print("Radio initialized")

    def send(self, data: bytes):
        self.nrf.stop_listening()
        try:
            self.nrf.send(data)
        except OSError as e:
            print("Send failed:", e)
        self.nrf.start_listening()

    def available(self):
        return self.nrf.any()

    def recv(self):
        if self.nrf.any():
            return self.nrf.recv()
        return None