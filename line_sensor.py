from machine import Pin, ADC
import time


class LineSensor:
    """Three-channel IR line sensor with white/black calibration."""

    def __init__(self, left_pin, centre_pin, right_pin):
        self.left_adc   = ADC(Pin(left_pin))
        self.centre_adc = ADC(Pin(centre_pin))
        self.right_adc  = ADC(Pin(right_pin))
        self.white = (0, 0, 0)
        self.black = (65535, 65535, 65535)

    def calibrate(self, status_cb=None, samples=100):
        """Sample white then black surfaces. status_cb(list[str]) shows prompts."""
        results = {}
        for colour, msg in (("WHITE", "Place on WHITE"), ("BLACK", "Place on BLACK")):
            if status_cb:
                status_cb([msg, "then wait..."])
            for remaining in (3, 2, 1):
                if status_cb:
                    status_cb([msg, "", "    {}".format(remaining)])
                time.sleep(1)

            l = c = r = 0
            for i in range(samples):
                rl = self.left_adc.read_u16()
                rc = self.centre_adc.read_u16()
                rr = self.right_adc.read_u16()
                l += rl; c += rc; r += rr
                if i % 10 == 0 and status_cb:
                    status_cb([
                        "Reading " + colour, "",
                        "L:{}".format(rl),
                        "C:{}".format(rc),
                        "R:{}".format(rr),
                    ])
                time.sleep(0.01)
            l //= samples; c //= samples; r //= samples

            if status_cb:
                status_cb([colour + " done", "",
                           "L:{}".format(l), "C:{}".format(c), "R:{}".format(r)])
            time.sleep(1.5)

            results[colour] = (l, c, r)

        self.white = results["WHITE"]
        self.black = results["BLACK"]
        print("White:", *self.white)
        print("Black:", *self.black)

    @staticmethod
    def _scale(raw, white_val, black_val):
        raw = min(max(raw, white_val), black_val)
        return (raw - white_val) * 100 // (black_val - white_val)

    def get_position(self):
        """Return (position [-100..100] or None, left, centre, right) scaled 0-100."""
        wl, wc, wr = self.white
        bl, bc, br = self.black
        left   = self._scale(self.left_adc.read_u16(),   wl, bl)
        centre = self._scale(self.centre_adc.read_u16(), wc, bc)
        right  = self._scale(self.right_adc.read_u16(),  wr, br)

        total = left + centre + right
        if total < 5:
            return None, left, centre, right

        position = (-left + right) * 100 // total
        position = min(max(position, -100), 100)
        return position, left, centre, right
