from machine import Pin, I2C
from ssd1306 import SSD1306_I2C


class Display:
    """SSD1306 OLED wrapper that prints to console if hardware is missing."""

    def __init__(self, sda_pin, scl_pin, width=128, height=64, freq=200000, bus=0):
        self.width = width
        self.height = height
        i2c = I2C(bus, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
        try:
            self.oled = SSD1306_I2C(width, height, i2c)
            self.has_oled = True
        except Exception:
            self.oled = None
            self.has_oled = False
            print("OLED not found, continuing without display")

    def show(self, lines):
        """Render up to 6 lines, or print them if no OLED is attached."""
        if not self.has_oled:
            for line in lines:
                print(line)
            return
        self.oled.fill(0)
        for i, text in enumerate(lines[:6]):
            self.oled.text(text, 0, i * 10)
        self.oled.show()
