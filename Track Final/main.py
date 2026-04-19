"""
Smooth proportional steering + hallway navigation + front-wall avoidance.

Line-following as before, but now:
  * Front ultrasonic triggers a reverse-and-180 manoeuvre if a wall appears
    within ~15 cm.
  * When the line is lost, the robot stops for 2 s and checks the side
    ultrasonics. If both walls are within hallway range (handles the 300 mm
    straight hallway and the 200-400 mm tapered one), it enters HALLWAY
    mode and wall-centres instead of pivot-searching.
  * Hallway centring is slow and capped so the robot cannot over-steer into
    a wall.
  * Hallway centring is suppressed during the avoidance spin so losing the
    line mid-180 doesn't kick off a bogus wall-centre.

Pins:
  Motors  : left  IN1=7  IN2=8  EN=6   ; right IN1=9  IN2=10 EN=11
  IR      : left=28 centre=27 right=26
  OLED    : SDA=16 SCL=17
  Sonars  : left  TRIG=5  ECHO=4
            front TRIG=14 ECHO=15
            right TRIG=12 ECHO=13

Note: HC-SR04 ECHO is 5 V logic. Drop it to 3.3 V with a divider
(e.g. 1 kΩ / 2 kΩ) before feeding the Pico input, or the Pico will brown
out or latch up over time.
"""

import time

from config import *
from motor import Motor
from display import Display
from ultrasonic import Ultrasonic
from line_sensor import LineSensor


# Hardware init.
left_motor  = Motor("left",  MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  MOTOR_LEFT_EN)
right_motor = Motor("right", MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN)

line = LineSensor(IR_LEFT, IR_CENTRE, IR_RIGHT)

us_left  = Ultrasonic(US_LEFT_TRIG,  US_LEFT_ECHO)
us_front = Ultrasonic(US_FRONT_TRIG, US_FRONT_ECHO)
us_right = Ultrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)

display = Display(OLED_SDA, OLED_SCL)


# Motor helpers.
def stop_motors():
    left_motor.stop()
    right_motor.stop()


def drive(left_dir, right_dir, left_d, right_d):
    """left_dir/right_dir: 1 forward, 0 backward. Duties are unsigned."""
    (left_motor.set_forwards  if left_dir  == 1 else left_motor.set_backwards)()
    (right_motor.set_forwards if right_dir == 1 else right_motor.set_backwards)()
    left_motor.duty(max(left_d,  MIN_DUTY))
    right_motor.duty(max(right_d, MIN_DUTY))


# Calibration.
line.calibrate(status_cb=display.show)


# Kick-start.
display.show(["Starting...", "", "Place on line!"])
time.sleep(2)
display.show(["GO!"])

left_motor.set_forwards()
right_motor.set_forwards()
left_motor.duty(50)
right_motor.duty(50)
time.sleep(0.1)


# State machine.
last_direction   = "left"
search_counter   = 0
smooth_pos       = 0
last_left_duty   = BASE_SPEED
last_right_duty  = BASE_SPEED
last_oled_ms     = 0
front_poll_ms    = 0
front_dist       = None
side_left_dist   = None
side_right_dist  = None
hallway_lost_ct  = 0
state            = "TRACK"
avoid_phase      = ""
avoid_started_ms = 0


try:
    while True:
        now = time.ticks_ms()
        position, s_left, s_centre, s_right = line.get_position()

        # --- Front-wall check ---------------------------------------------
        # Suppressed during AVOID so we don't retrigger while reversing.
        if state != "AVOID" and time.ticks_diff(now, front_poll_ms) >= FRONT_POLL_INTERVAL_MS:
            front_poll_ms = now
            front_dist = us_front.read()
            if front_dist is not None and 20 <= front_dist <= FRONT_STOP_MM:
                stop_motors()
                state = "AVOID"
                avoid_phase = "REVERSE"
                avoid_started_ms = now

        # --- TRACK --------------------------------------------------------
        if state == "TRACK":
            if position is not None:
                smooth_pos = (smooth_pos * 2 + position) // 3
                abs_smooth = abs(smooth_pos)

                correction = 0 if abs_smooth < DEAD_ZONE else (smooth_pos * KP) // 100
                speed = BASE_SPEED - CORNER_SPEED_CUT if abs_smooth > CORNER_THRESHOLD else BASE_SPEED

                left_speed  = speed + correction
                right_speed = speed - correction

                left_dir, right_dir = 1, 1
                if left_speed  < 0: left_dir, left_speed  = 0, -left_speed
                if right_speed < 0: right_dir, right_speed = 0, -right_speed

                last_left_duty  = max(left_speed,  MIN_DUTY)
                last_right_duty = max(right_speed, MIN_DUTY)
                drive(left_dir, right_dir, last_left_duty, last_right_duty)

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
                # Line just went missing. Brief coast, then stop and look
                # for walls to decide between hallway and search.
                search_counter += 1
                if search_counter <= COAST_CYCLES:
                    left_motor.duty(last_left_duty)
                    right_motor.duty(last_right_duty)
                else:
                    stop_motors()
                    display.show(["LINE LOST", "", "Checking walls..."])
                    time.sleep_ms(LINE_LOST_PAUSE_MS)

                    side_left_dist  = us_left.read_avg()
                    side_right_dist = us_right.read_avg()

                    in_hallway = (
                        side_left_dist  is not None and side_left_dist  <= HALLWAY_MAX_MM and
                        side_right_dist is not None and side_right_dist <= HALLWAY_MAX_MM
                    )

                    smooth_pos = 0
                    hallway_lost_ct = 0
                    state = "HALLWAY" if in_hallway else "SEARCH"

        # --- HALLWAY ------------------------------------------------------
        elif state == "HALLWAY":
            # Line reappears -> hand back to normal tracking.
            if position is not None:
                state = "TRACK"
                search_counter = 0
                continue

            side_left_dist  = us_left.read_avg(n=2)
            side_right_dist = us_right.read_avg(n=2)

            walls_ok = (
                side_left_dist  is not None and side_left_dist  <= HALLWAY_MAX_MM and
                side_right_dist is not None and side_right_dist <= HALLWAY_MAX_MM
            )

            if not walls_ok:
                hallway_lost_ct += 1
                if hallway_lost_ct > HALLWAY_LOST_LIMIT:
                    # Walls gone for a while - fall back to pivot search.
                    state = "SEARCH"
                    continue
                # Briefly: just keep going straight at hallway speed.
                drive(1, 1, HALLWAY_SPEED, HALLWAY_SPEED)
            else:
                hallway_lost_ct = 0
                # diff > 0 means the right wall is farther -> we're closer
                # to the LEFT wall -> veer RIGHT (boost left, ease right).
                diff = side_right_dist - side_left_dist
                if abs(diff) < HALLWAY_DEAD_MM:
                    correction = 0
                else:
                    correction = (diff * HALLWAY_CORR_GAIN) // 100
                    if correction >  HALLWAY_MAX_CORR: correction =  HALLWAY_MAX_CORR
                    if correction < -HALLWAY_MAX_CORR: correction = -HALLWAY_MAX_CORR

                drive(1, 1,
                      HALLWAY_SPEED + correction,
                      HALLWAY_SPEED - correction)

        # --- SEARCH (original pivot recovery) -----------------------------
        elif state == "SEARCH":
            if position is not None:
                state = "TRACK"
                search_counter = 0
                continue
            if last_direction == "left":
                left_motor.stop()
                right_motor.set_forwards()
                right_motor.duty(SEARCH_SPEED)
            else:
                left_motor.set_forwards()
                right_motor.stop()
                left_motor.duty(SEARCH_SPEED)

        # --- AVOID (reverse, then spin ~180) ------------------------------
        # Line-loss handling is disabled here by construction: we never
        # look at `position` while state == "AVOID", so hallway mode can't
        # be triggered by the line vanishing during the spin.
        elif state == "AVOID":
            elapsed = time.ticks_diff(now, avoid_started_ms)
            if avoid_phase == "REVERSE":
                drive(0, 0, AVOID_SPEED, AVOID_SPEED)
                if elapsed >= AVOID_REVERSE_MS:
                    avoid_phase = "TURN"
                    avoid_started_ms = now
            elif avoid_phase == "TURN":
                # Spin in place: left wheel back, right wheel forward.
                drive(0, 1, AVOID_SPEED, AVOID_SPEED)
                if elapsed >= AVOID_TURN_MS:
                    stop_motors()
                    state = "TRACK"
                    avoid_phase = ""
                    search_counter = 0
                    smooth_pos = 0

        # --- OLED ---------------------------------------------------------
        if time.ticks_diff(now, last_oled_ms) >= OLED_UPDATE_MS:
            last_oled_ms = now
            try:
                display.show([
                    state + (" " + avoid_phase if state == "AVOID" else ""),
                    "L:{} C:{} R:{}".format(s_left, s_centre, s_right),
                    "pos:{} sm:{}".format(position if position is not None else "-", smooth_pos),
                    "F:{}mm".format(front_dist if front_dist is not None else "-"),
                    "LW:{} RW:{}".format(
                        side_left_dist  if side_left_dist  is not None else "-",
                        side_right_dist if side_right_dist is not None else "-",
                    ),
                ])
            except Exception:
                pass

        time.sleep(LOOP_DELAY)

except Exception as e:
    stop_motors()
    print("CRASH:", e)
