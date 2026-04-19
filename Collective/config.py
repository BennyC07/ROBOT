# Pin configuration.
MOTOR_LEFT_IN1  = 7
MOTOR_LEFT_IN2  = 8
MOTOR_LEFT_EN   = 6
MOTOR_RIGHT_IN1 = 9
MOTOR_RIGHT_IN2 = 10
MOTOR_RIGHT_EN  = 11
IR_LEFT         = 28
IR_CENTRE       = 27
IR_RIGHT        = 26
OLED_SDA        = 16
OLED_SCL        = 17

US_LEFT_TRIG    = 5
US_LEFT_ECHO    = 4
US_FRONT_TRIG   = 14
US_FRONT_ECHO   = 15
US_RIGHT_TRIG   = 12
US_RIGHT_ECHO   = 13

# Line-following tuning.
BASE_SPEED       = 22
KP               = 25
MIN_DUTY         = 17
SEARCH_SPEED     = 25
DEAD_ZONE        = 3
# Position low-pass: higher = more dampening, slower response.
# new_smooth = (smooth * SMOOTH_WEIGHT + raw) / (SMOOTH_WEIGHT + 1)
SMOOTH_WEIGHT    = 4
CORNER_THRESHOLD = 35
CORNER_SPEED_CUT = 8
LOOP_DELAY       = 0.005
OLED_UPDATE_MS   = 300

# Coast-before-hallway-check.
COAST_CYCLES        = 60

# Hallway-navigation tuning.
HALLWAY_MAX_MM      = 450
HALLWAY_SPEED       = 18
HALLWAY_CORR_GAIN   = 15
HALLWAY_DEAD_MM     = 25
HALLWAY_MAX_CORR    = 8
HALLWAY_LOST_LIMIT  = 40
SIDE_POLL_INTERVAL_MS = 150

# Front-wall avoidance.
FRONT_STOP_MM          = 150
FRONT_POLL_INTERVAL_MS = 80
AVOID_REVERSE_MS       = 600
AVOID_TURN_MS          = 1400
AVOID_SPEED            = 30
