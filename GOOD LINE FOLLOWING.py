from machine import Pin, PWM, ADC
from time import sleep

# =========================
# Motor setup
# =========================
ENA = PWM(Pin(6))
IN1 = Pin(7,  Pin.OUT)
IN2 = Pin(8,  Pin.OUT)
IN3 = Pin(9,  Pin.OUT)
IN4 = Pin(10, Pin.OUT)
ENB = PWM(Pin(11))

ENA.freq(1000)
ENB.freq(1000)

# =========================
# IR sensors (digital)
# =========================
left_sensor   = Pin(21, Pin.IN)
centre_sensor = Pin(20, Pin.IN)
right_sensor  = Pin(19, Pin.IN)

# =========================
# Speed settings
# =========================
SPEED       = 30000   # 0 – 65535
TURN_SPEED  = 25000

# =========================
# Motor functions
# =========================
def forward():
    IN1.on();  IN2.off()
    IN3.on();  IN4.off()
    ENA.duty_u16(SPEED)
    ENB.duty_u16(SPEED)

def turn_right():
    IN1.on();  IN2.off()
    IN3.off(); IN4.on()
    ENA.duty_u16(TURN_SPEED)
    ENB.duty_u16(TURN_SPEED)

def turn_left():
    IN1.off(); IN2.on()
    IN3.on();  IN4.off()
    ENA.duty_u16(TURN_SPEED)
    ENB.duty_u16(TURN_SPEED)

def stop():
    IN1.off(); IN2.off()
    IN3.off(); IN4.off()
    ENA.duty_u16(0)       # also zero the enable pins so motors actually stop
    ENB.duty_u16(0)

# =========================
# Sensor read
# =========================
def read_sensors():
    return [left_sensor.value(), centre_sensor.value(), right_sensor.value()]

# =========================
# State (lives outside robot()
# so it persists across calls)
# =========================
last_direction = "forward"   # "left" | "right" | "forward"

# =========================
# Main robot logic
# =========================
def robot():
    global last_direction

    sv = read_sensors()

    # All three on = junction/roundabout — go straight through
    if sv == [1, 1, 1]:
        forward()

    # Centre only — straight
    elif sv == [0, 1, 0]:
        forward()
        last_direction = "forward"

    # Left sensors — turn left
    elif sv == [1, 0, 0] or sv == [1, 1, 0]:
        turn_left()
        last_direction = "left"

    # Right sensors — turn right
    elif sv == [0, 0, 1] or sv == [0, 1, 1]:
        turn_right()
        last_direction = "right"

    # All off — lost the line, use last known direction to search
    elif sv == [0, 0, 0]:
        if last_direction == "left":
            turn_left()          # spin back toward where line last was
        elif last_direction == "right":
            turn_right()
        else:
            forward()            # was going straight, creep forward

    sleep(0.05)

# =========================
# Entry point
# =========================
if __name__ == '__main__':
    try:
        while True:
            robot()
    except KeyboardInterrupt:
        stop()