"""
Smooth proportional Steering

Calibration and status display on an SSD1306 OLED.
Reads three IR sensors -> calculates line position ->  drives two motors with
damped proportional control.
Requires ssd1306.py and motor.py on the Pico. Upload this file as main.py.

Upload steps (run each in Terminal, replace the port with yours from
`ls /dev/tty.usbmodem*`):

    pip install mpremote
    mpremote connect /dev/tty.usbmodem1101 exec "import os; [os.remove(f) for f in os.listdir()]"
    mpremote connect /dev/tty.usbmodem1101 cp ssd1306.py :ssd1306.py
    mpremote connect /dev/tty.usbmodem1101 cp motor.py :motor.py
    mpremote connect /dev/tty.usbmodem1101 cp main.py :main.py
    mpremote connect /dev/tty.usbmodem1101 reset
"""

from machine import Pin, ADC, I2C
import time

from ssd1306 import SSD1306_I2C
from motor import Motor


# Pin configuration — update to match your wiring.
MOTOR_LEFT_IN1  = 7
MOTOR_LEFT_IN2  = 8
MOTOR_LEFT_EN   = 6
MOTOR_RIGHT_IN1 =9
MOTOR_RIGHT_IN2 = 10
MOTOR_RIGHT_EN  = 11
IR_LEFT         = 28
IR_CENTRE       =27
IR_RIGHT        =26
OLED_SDA        = 16
OLED_SCL        = 17

# Control tuning.
BASE_SPEED       = 22    # motor duty on a straight line
KP               = 25    # proportional gain
MIN_DUTY         = 10    # floor for motor duty while driving
SEARCH_SPEED     = 25    # pivot speed when line is lost
DEAD_ZONE        = 3     # ignore corrections below this magnitude
CORNER_THRESHOLD = 35    # |smooth_pos| above this counts as a sharp corner
CORNER_SPEED_CUT = 8     # amount to drop speed on sharp corners
LOOP_DELAY       = 0.005
OLED_UPDATE_MS   = 300


# Hardware init.
left_motor  = Motor("left", MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN)
right_motor = Motor("right", MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN)

left_adc   = ADC(Pin(IR_LEFT))
centre_adc = ADC(Pin(IR_CENTRE))
right_adc  = ADC(Pin(IR_RIGHT))

i2c = I2C(0, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=200000)
try:
    oled = SSD1306_I2C(128, 64, i2c)
    has_oled = True
except Exception:
    has_oled = False
    print("OLED not found, continuing without display")


def oled_msg(lines):
    """Draw up to 6 lines of text; falls back to print() if no OLED."""
    if not has_oled:
        for line in lines:
            print(line)
        return
    oled.fill(0)
    for i, text in enumerate(lines[:6]):
        oled.text(text, 0, i * 10)
    oled.show()


def calibrate():
    """Sample each sensor on white then black, displaying progress on the OLED."""
    samples = 100

    for colour, msg in (("WHITE", "Place on WHITE"), ("BLACK", "Place on BLACK")):
        oled_msg([msg, "then wait..."])
        for remaining in (3, 2, 1):
            oled_msg([msg, "", "    {}".format(remaining)])
            time.sleep(1)

        l = c = r = 0
        for i in range(samples):
            rl = left_adc.read_u16()
            rc = centre_adc.read_u16()
            rr = right_adc.read_u16()
            l += rl; c += rc; r += rr
            if i % 10 == 0:
                oled_msg([
                    "Reading " + colour,
                    "",
                    "L:{}".format(rl),
                    "C:{}".format(rc),
                    "R:{}".format(rr),
                ])
            time.sleep(0.01)
        l //= samples; c //= samples; r //= samples

        oled_msg([colour + " done", "", "L:{}".format(l), "C:{}".format(c), "R:{}".format(r)])
        time.sleep(1.5)

        if colour == "WHITE":
            wl, wc, wr = l, c, r
        else:
            bl, bc, br = l, c, r

    print("White:", wl, wc, wr)
    print("Black:", bl, bc, br)
    return wl, wc, wr, bl, bc, br


white_left, white_centre, white_right, black_left, black_centre, black_right = calibrate()


def scale(raw, white_val, black_val):
    """Map a raw ADC reading to 0 (white) – 100 (black)."""
    raw = min(max(raw, white_val), black_val)
    return (raw - white_val) * 100 // (black_val - white_val)


def get_line_position():
    """Return (position, left, centre, right). Position is -100..+100, or None if no line."""
    left   = scale(left_adc.read_u16(),   white_left,   black_left)
    centre = scale(centre_adc.read_u16(), white_centre, black_centre)
    right  = scale(right_adc.read_u16(),  white_right,  black_right)

    total = left + centre + right
    if total < 5:
        return None, left, centre, right

    position = (-left + right) * 100 // total
    position = min(max(position, -100), 100)
    return position, left, centre, right


# Brief kick-start so the car isn't dead still when the loop begins.
oled_msg(["Starting...", "", "Place on line!"])
time.sleep(2)
oled_msg(["GO!"])

left_motor.set_forwards()
right_motor.set_forwards()
left_motor.duty(50)
right_motor.duty(50)
time.sleep(0.1)


# Main control loop.
last_direction  = "left"
search_counter  = 0
smooth_pos      = 0
last_left_duty  = BASE_SPEED
last_right_duty = BASE_SPEED
last_oled_ms    = 0
state           = "TRACK"

try:
    while True:
        position, s_left, s_centre, s_right = get_line_position()

        if position is not None:
            # EMA low-pass: 2/3 history, 1/3 new reading.
            smooth_pos = (smooth_pos * 2 + position) // 3
            abs_smooth = abs(smooth_pos)

            if abs_smooth < DEAD_ZONE:
                correction = 0
            else:
                correction = (smooth_pos * KP) // 100

            # Drop speed on sharp corners to avoid overshoot.
            if abs_smooth > CORNER_THRESHOLD:
                speed = BASE_SPEED - CORNER_SPEED_CUT
            else:
                speed = BASE_SPEED

            left_speed  = speed + correction
            right_speed = speed - correction

            left_motor.set_forwards()
            right_motor.set_forwards()

            # A strong correction can push a wheel negative — reverse it instead.
            if left_speed < 0:
                left_motor.set_backwards()
                left_speed = -left_speed
            if right_speed < 0:
                right_motor.set_backwards()
                right_speed = -right_speed

            last_left_duty  = max(left_speed, MIN_DUTY)
            last_right_duty = max(right_speed, MIN_DUTY)
            left_motor.duty(last_left_duty)
            right_motor.duty(last_right_duty)

            # Remember which side the line was on for recovery.
            if position < -10:
                last_direction = "left"
            elif position > 10:
                last_direction = "right"
            elif s_left > s_right + 5:
                last_direction = "left"
            elif s_right > s_left + 5:
                last_direction = "right"

            search_counter = 0
            state = "TRACK"

        else:
            # Line lost — coast briefly, then pivot toward last-known side until reacquired.
            search_counter += 1
            if search_counter <= 20:
                left_motor.duty(last_left_duty)
                right_motor.duty(last_right_duty)
                state = "COAST"
            else:
                smooth_pos = 0
                if last_direction == "left":
                    left_motor.stop()
                    right_motor.set_forwards()
                    right_motor.duty(SEARCH_SPEED)
                    state = "SEARCH<"
                else:
                    left_motor.set_forwards()
                    right_motor.stop()
                    left_motor.duty(SEARCH_SPEED)
                    state = "SEARCH>"

        # Refresh the OLED status at a fixed rate so it doesn't stall the control loop.
        now = time.ticks_ms()
        if time.ticks_diff(now, last_oled_ms) >= OLED_UPDATE_MS:
            last_oled_ms = now
            try:
                oled_msg([
                    state,
                    "L:{} C:{} R:{}".format(s_left, s_centre, s_right),
                    "pos: {}".format(position if position is not None else "---"),
                    "sm:  {}".format(smooth_pos),
                    "dir: {}".format(last_direction),
                ])
            except Exception:
                pass

        time.sleep(LOOP_DELAY)

except Exception as e:
    left_motor.stop()
    right_motor.stop()
    print("CRASH:", e)