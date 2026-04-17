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

#Ultrasonic Sensors

TRIGC = Pin(14, Pin.OUT)   # centre — forward-facing
ECHOC = Pin(15, Pin.IN)

TRIGL = Pin(5,  Pin.OUT)   # left — side-facing
ECHOL = Pin(4,  Pin.IN)

TRIGR = Pin(12, Pin.OUT)   # right — side-facing
ECHOR = Pin(13, Pin.IN)

def dist(trig, echo):
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()
    timeout = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), timeout) > 30000:
            return None
    start = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), timeout) > 30000:
            return None
    end = time.ticks_us()
    return (0.343 * time.ticks_diff(end, start)) / 2

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
    dl = dist(TRIGL, ECHOL)
    dr = dist(TRIGR, ECHOR)
    dc = dist(TRIGC, ECHOC)

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
        stop()
        time.sleep(1)
        if dl < 40 and dr < 40:
            if dl - 
            if dr > dr:
                turn_left()
        # was going straight, creep forward

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