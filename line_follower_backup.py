# ============================================================
# LINE FOLLOWING CAR - SMOOTH PROPORTIONAL CONTROL
# ============================================================
# BACKUP - DO NOT MODIFY - This is the working line follower code
# ============================================================
from machine import Pin, PWM, ADC
import time

# --- Motor Class ---
class Motor:
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
            self.IN1.on()
            self.IN2.off()
        else:
            self.IN1.off()
            self.IN2.on()

    def set_backwards(self):
        if self.side == "left":
            self.IN1.off()
            self.IN2.on()
        else:
            self.IN1.on()
            self.IN2.off()

    def stop(self):
        self.EN.duty_u16(0)

# --- Encoder Class ---
class Encoder:
    def __init__(self, pin_number):
        self.pin = Pin(pin_number, Pin.IN)
        self._count = 0
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=self._callback)

    def _callback(self, pin):
        self._count += 1

    def get_count(self):
        return self._count

    def clear_count(self):
        self._count = 0

# --- Create Hardware ---
left_motor = Motor("left", 11, 10, 7)
right_motor = Motor("right", 8, 9, 6)

left_encoder = Encoder(18)
right_encoder = Encoder(19)

left_adc = ADC(Pin(28))
centre_adc = ADC(Pin(27))
right_adc = ADC(Pin(26))

# --- Auto-Calibration ---
def calibrate():
    print("CALIBRATION: Place all sensors on WHITE surface")
    print("Reading in 3 seconds...")
    time.sleep(3)

    wl, wc, wr = 0, 0, 0
    for i in range(100):
        wl += left_adc.read_u16()
        wc += centre_adc.read_u16()
        wr += right_adc.read_u16()
        time.sleep(0.01)
    wl //= 100
    wc //= 100
    wr //= 100

    print("White readings:", wl, wc, wr)
    print("Now place all sensors on BLACK line")
    print("Reading in 3 seconds...")
    time.sleep(3)

    bl, bc, br = 0, 0, 0
    for i in range(100):
        bl += left_adc.read_u16()
        bc += centre_adc.read_u16()
        br += right_adc.read_u16()
        time.sleep(0.01)
    bl //= 100
    bc //= 100
    br //= 100

    print("Black readings:", bl, bc, br)
    return wl, wc, wr, bl, bc, br

white_left, white_centre, white_right, black_left, black_centre, black_right = calibrate()

# --- Scaling Function ---
def scale(raw, white_val, black_val):
    raw = min(max(raw, white_val), black_val)
    return (raw - white_val) * 100 // (black_val - white_val)

# --- Speed Settings ---
BASE_SPEED = 25
KP = 17
MIN_DUTY = 10
SEARCH_SPEED = 25
SEARCH_TIMEOUT = 10000
LOOP_DELAY = 0.005

# --- Position Calculation ---
def get_line_position():
    raw_left = left_adc.read_u16()
    raw_centre = centre_adc.read_u16()
    raw_right = right_adc.read_u16()

    left = scale(raw_left, white_left, black_left)
    centre = scale(raw_centre, white_centre, black_centre)
    right = scale(raw_right, white_right, black_right)

    total = left + centre + right
    if total < 5:
        return None, left, centre, right

    position = (-left + right) * 100 // total
    position = min(max(position, -100), 100)
    return position, left, centre, right

# --- Main Loop ---
last_direction = "left"
search_counter = 0
smooth_pos = 0
last_left_duty = BASE_SPEED
last_right_duty = BASE_SPEED

print("Starting in 2 seconds — place car on the line!")
time.sleep(2)
print("GO!")

left_motor.set_forwards()
right_motor.set_forwards()
left_motor.duty(50)
right_motor.duty(50)
time.sleep(0.1)

loop_count = 0

try:
    while True:
        position, s_left, s_centre, s_right = get_line_position()

        if position is not None:
            smooth_pos = position

            abs_smooth = abs(smooth_pos)
            if abs_smooth < 6:
                correction = 0
            else:
                correction = (smooth_pos * KP) // 100
            speed = BASE_SPEED

            left_speed = speed + correction
            right_speed = speed - correction

            left_motor.set_forwards()
            right_motor.set_forwards()

            if left_speed < 0:
                left_motor.set_backwards()
                left_speed = -left_speed
            if right_speed < 0:
                right_motor.set_backwards()
                right_speed = -right_speed

            last_left_duty = max(left_speed, MIN_DUTY)
            last_right_duty = max(right_speed, MIN_DUTY)
            left_motor.duty(last_left_duty)
            right_motor.duty(last_right_duty)

            if position < -10:
                last_direction = "left"
            elif position > 10:
                last_direction = "right"
            elif s_left > s_right + 5:
                last_direction = "left"
            elif s_right > s_left + 5:
                last_direction = "right"

            search_counter = 0

        else:
            search_counter += 1

            if search_counter <= 20:
                left_motor.duty(last_left_duty)
                right_motor.duty(last_right_duty)
            elif search_counter > SEARCH_TIMEOUT:
                left_motor.stop()
                right_motor.stop()
            else:
                smooth_pos = 0
                if last_direction == "left":
                    left_motor.stop()
                    right_motor.set_forwards()
                    right_motor.duty(SEARCH_SPEED)
                else:
                    left_motor.set_forwards()
                    right_motor.stop()
                    left_motor.duty(SEARCH_SPEED)

        loop_count += 1
        if loop_count % 200 == 0:
            print("pos:", position, "sm:", smooth_pos, "L:", s_left, "C:", s_centre, "R:", s_right, "sc:", search_counter)

        time.sleep(LOOP_DELAY)

except Exception as e:
    left_motor.stop()
    right_motor.stop()
    print("CRASH:", e)
