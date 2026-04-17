from machine import Pin, PWM
class Motor:
    """Dual-input H-bridge motor with PWM speed control."""

    def __init__(self, side, in1_pin, in2_pin, en_pin):
        self.side = side
        self.IN1 = Pin(in1_pin, Pin.OUT)
        self.IN2 = Pin(in2_pin, Pin.OUT)
        self.EN = PWM(Pin(en_pin))
        self.EN.freq(1000)

    def duty(self, pwm):
        pwm = min(max(pwm, 0), 100)
        self.EN.duty_u16(int(655 * pwm))

    def set_forwards(self):
        if self.side == "left":
            self.IN1.on(); self.IN2.off()
        else:
            self.IN1.off(); self.IN2.on()

    def set_backwards(self):
        if self.side == "left":
            self.IN1.off(); self.IN2.on()
        else:
            self.IN1.on(); self.IN2.off()

    def stop(self):
        self.EN.duty_u16(0)