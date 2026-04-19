from machine import Pin, time_pulse_us
import time


class Ultrasonic:
    """HC-SR04 distance sensor. Returns distances in mm."""

    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.low()

    def read(self, timeout_us=25000):
        """Fire one ping and return mm, or None on timeout."""
        self.trig.low()
        time.sleep_us(2)
        self.trig.high()
        time.sleep_us(10)
        self.trig.low()
        try:
            duration = time_pulse_us(self.echo, 1, timeout_us)
        except OSError:
            return None
        if duration < 0:
            return None
        # Speed of sound ~343 m/s, round trip: mm = duration_us * 10 / 58.
        return duration * 10 // 58

    def read_avg(self, n=3):
        """Average n pings; drops timeouts and impossible values."""
        total = 0
        count = 0
        for _ in range(n):
            d = self.read()
            if d is not None and 20 <= d <= 2000:
                total += d
                count += 1
            time.sleep_ms(5)
        return total // count if count else None
