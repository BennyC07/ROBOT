"""
RC Car — WiFi remote control with collision avoidance and OLED display.
Upload to Pico W as main.py. Requires ssd1306.py on the board.

HOW TO UPLOAD (run these in Terminal one at a time):

    1. Install the upload tool (only need to do this once ever):
       pip install mpremote

    2. Plug your Pico W into USB. Find its port:
       ls /dev/tty.usbmodem*
       (it'll print something like /dev/tty.usbmodem1101 — use that below)

    3. Wipe the Pico clean:
       mpremote connect /dev/tty.usbmodem1101 exec "import os; [os.remove(f) for f in os.listdir()]"

    4. Upload the OLED driver:
       mpremote connect /dev/tty.usbmodem1101 cp ssd1306.py :ssd1306.py

    5. Upload this file as main.py (so it runs automatically on boot):
       mpremote connect /dev/tty.usbmodem1101 cp rc_car.py :main.py

    6. Restart the Pico:
       mpremote connect /dev/tty.usbmodem1101 reset

    7. Connect your laptop to the "PicoRobot" WiFi (password: robot1234),
       open http://192.168.4.1 in your browser, and drive with arrow keys.
"""

from machine import Pin, PWM, ADC, I2C
import machine as mach
import network
import socket
import json
import time

from ssd1306 import SSD1306_I2C


# Pin Configuration
# Modify these to match your wiring
MOTOR_LEFT_IN1  = 11
MOTOR_LEFT_IN2  = 10
MOTOR_LEFT_EN   = 7
MOTOR_RIGHT_IN1 = 8
MOTOR_RIGHT_IN2 = 9
MOTOR_RIGHT_EN  = 6
ENCODER_LEFT    = 18
ENCODER_RIGHT   = 19
IR_LEFT         = 28
IR_CENTRE       = 27
IR_RIGHT        = 26
OLED_SDA        = 12
OLED_SCL        = 13
US_TRIG         = 4
US_ECHO         = 5
SERVO_PIN       = 15

# Tuning
DRIVE_SPEED    = 30
KP             = 17
MIN_DUTY       = 10
PIVOT_SPEED    = 25
SAFE_DISTANCE  = 150   # mm, forward blocked below this
CMD_TIMEOUT_MS = 600   # ms, motors stop if no command received

# WiFi
AP_SSID = "PicoRobot"
AP_PASS = "robot1234"

VALID_CMDS = {"S", "F", "B", "L", "R", "FL", "FR", "BL", "BR"}


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


# Hardware

left_motor  = Motor("left", MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN)
right_motor = Motor("right", MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN)

left_encoder  = Encoder(ENCODER_LEFT)
right_encoder = Encoder(ENCODER_RIGHT)

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

us_trig = Pin(US_TRIG, Pin.OUT)
us_echo = Pin(US_ECHO, Pin.IN)
us_trig.value(0)

servo = PWM(Pin(SERVO_PIN))
servo.freq(50)


def servo_angle(angle):
    pulse_us = 500 + (2500 - 500) * angle // 180
    servo.duty_u16(pulse_us * 65535 // 20000)

servo_angle(90)


# Ultrasonic

def read_distance_mm():
    us_trig.value(0)
    time.sleep_us(5)
    us_trig.value(1)
    time.sleep_us(10)
    us_trig.value(0)
    duration = mach.time_pulse_us(us_echo, 1, 30000)
    if duration < 0:
        return -1
    return duration * 17 // 100


# Encoder Speed

_last_l = 0
_last_r = 0
_last_spd_ms = time.ticks_ms()
left_tps = 0
right_tps = 0

def update_wheel_speeds():
    global _last_l, _last_r, _last_spd_ms, left_tps, right_tps
    now = time.ticks_ms()
    dt = time.ticks_diff(now, _last_spd_ms)
    if dt < 200:
        return
    lc = left_encoder.get_count()
    rc = right_encoder.get_count()
    left_tps  = (lc - _last_l) * 1000 // dt
    right_tps = (rc - _last_r) * 1000 // dt
    _last_l = lc
    _last_r = rc
    _last_spd_ms = now


# IR Sensors

def read_ir():
    l = left_adc.read_u16()   * 100 // 65535
    c = centre_adc.read_u16() * 100 // 65535
    r = right_adc.read_u16()  * 100 // 65535
    return l, c, r


# Drive Control (proportional steering via EMA filter)

smooth_pos = 0

def apply_drive(cmd, dist_mm):
    global smooth_pos

    blocked = 0 < dist_mm < SAFE_DISTANCE

    if cmd == "S":
        smooth_pos = 0
        left_motor.stop()
        right_motor.stop()
        return blocked

    if cmd in ("F", "FL", "FR") and blocked:
        smooth_pos = 0
        left_motor.stop()
        right_motor.stop()
        return True

    # Pivot in place
    if cmd == "L":
        smooth_pos = 0
        left_motor.set_backwards()
        right_motor.set_forwards()
        left_motor.duty(PIVOT_SPEED)
        right_motor.duty(PIVOT_SPEED)
        return blocked

    if cmd == "R":
        smooth_pos = 0
        left_motor.set_forwards()
        right_motor.set_backwards()
        left_motor.duty(PIVOT_SPEED)
        right_motor.duty(PIVOT_SPEED)
        return blocked

    # Forward/backward with proportional steering
    if cmd in ("FL", "BL"):
        target = -70
    elif cmd in ("FR", "BR"):
        target = 70
    else:
        target = 0

    smooth_pos = (smooth_pos * 3 + target) // 4
    correction = (smooth_pos * KP) // 100

    left_speed  = DRIVE_SPEED + correction
    right_speed = DRIVE_SPEED - correction
    going_fwd = cmd in ("F", "FL", "FR")

    if going_fwd:
        left_motor.set_forwards()
        right_motor.set_forwards()
    else:
        left_motor.set_backwards()
        right_motor.set_backwards()

    if left_speed < 0:
        left_motor.set_backwards() if going_fwd else left_motor.set_forwards()
        left_speed = -left_speed

    if right_speed < 0:
        right_motor.set_backwards() if going_fwd else right_motor.set_forwards()
        right_speed = -right_speed

    left_motor.duty(max(left_speed, MIN_DUTY))
    right_motor.duty(max(right_speed, MIN_DUTY))
    return blocked


# OLED Display (alternates pages every 5s)

_last_oled_ms = 0

def update_oled(cmd, dist_mm, blocked, ir_l, ir_c, ir_r):
    global _last_oled_ms
    if not has_oled:
        return

    now = time.ticks_ms()
    if time.ticks_diff(now, _last_oled_ms) < 300:
        return
    _last_oled_ms = now

    page = (now // 5000) % 2
    oled.fill(0)

    if page == 0:
        oled.text("SENSORS", 36, 0)
        oled.text("IR L:{} C:{}".format(ir_l, ir_c), 0, 14)
        oled.text("   R:{}".format(ir_r), 0, 24)
        dist_str = "---" if dist_mm < 0 else "{}mm".format(dist_mm)
        oled.text("Dist: " + dist_str, 0, 36)
        oled.text("Dir: {}".format(cmd), 0, 48)
        if blocked and (now // 300) % 2:
            oled.text("BLOCKED!", 32, 56)
    else:
        oled.text("WHEELS", 40, 0)
        oled.text("L: {} t/s".format(left_tps), 0, 14)
        oled.text("R: {} t/s".format(right_tps), 0, 24)
        oled.text("L tot:{}".format(left_encoder.get_count()), 0, 36)
        oled.text("R tot:{}".format(right_encoder.get_count()), 0, 48)
        dist_str = "---" if dist_mm < 0 else "{}mm".format(dist_mm)
        oled.text("Dist: " + dist_str, 0, 56)

    oled.show()


# WiFi

def load_wifi():
    try:
        with open("wifi.json") as f:
            return json.load(f)
    except Exception:
        return None

def save_wifi(ssid, password):
    with open("wifi.json", "w") as f:
        json.dump({"ssid": ssid, "password": password}, f)

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    for _ in range(20):
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print("WiFi connected:", ip)
            return ip
        time.sleep(0.5)
    print("WiFi connection failed")
    return None

def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS)
    while not ap.active():
        time.sleep(0.1)
    ip = ap.ifconfig()[0]
    print("AP mode: {} pass:{} -> http://{}".format(AP_SSID, AP_PASS, ip))
    return ip


# Web Interface

HTML = b"""<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width">
<title>RC Car</title><style>
*{box-sizing:border-box}
body{background:#111;color:#fff;font-family:monospace;text-align:center;
padding:20px;user-select:none;margin:0}
h2{color:#0f0;margin:10px}
#dir{font-size:32px;margin:20px;color:#0ff}
#warn{color:#f44;font-size:22px;font-weight:bold;min-height:30px}
#dist{color:#aaa;margin:10px;font-size:16px}
.k{color:#555;margin-top:30px}
</style></head><body>
<h2>RC CAR</h2>
<div id="dir">STOPPED</div>
<div id="warn"></div>
<div id="dist"></div>
<div class="k">Arrow Keys to Drive</div>
<script>
var k={},L="S";
var N={S:"STOPPED",F:"FORWARD",B:"BACKWARD",L:"LEFT",R:"RIGHT",
FL:"FWD+LEFT",FR:"FWD+RIGHT",BL:"BACK+LEFT",BR:"BACK+RIGHT"};
function s(d){
if(d===L)return;L=d;
fetch("/cmd?d="+d).catch(function(){});
document.getElementById("dir").textContent=N[d]||d;
}
function u(){
var f=k[38],b=k[40],l=k[37],r=k[39],d="S";
if(f&&!b){d=l?"FL":r?"FR":"F";}
else if(b&&!f){d=l?"BL":r?"BR":"B";}
else if(l){d="L";}
else if(r){d="R";}
s(d);
}
document.onkeydown=function(e){
if(e.keyCode>=37&&e.keyCode<=40){e.preventDefault();k[e.keyCode]=1;u();}
};
document.onkeyup=function(e){
if(e.keyCode>=37&&e.keyCode<=40){e.preventDefault();delete k[e.keyCode];u();}
};
setInterval(function(){
fetch("/status").then(function(r){return r.json();}).then(function(d){
var dt=d.dist<0?"Dist: no echo":"Dist: "+d.dist+"mm";
document.getElementById("dist").textContent=dt;
document.getElementById("warn").textContent=d.blocked?"OBSTACLE DETECTED!":"";
}).catch(function(){});
},500);
</script></body></html>"""


# HTTP Server

current_cmd  = "S"
last_cmd_ms  = time.ticks_ms()
current_dist = -1
is_blocked   = False

def handle_http(conn):
    global current_cmd, last_cmd_ms, ap_mode, current_ip

    conn.settimeout(1.0)
    try:
        data = conn.recv(1024)
    except Exception:
        conn.close()
        return

    if not data:
        conn.close()
        return

    try:
        first_line = data.split(b"\r\n")[0].decode()
        method, path, _ = first_line.split(" ", 2)
    except Exception:
        conn.close()
        return

    if path == "/" and method == "GET":
        hdr = "HTTP/1.0 200 OK\r\nContent-Type:text/html\r\nContent-Length:{}\r\n\r\n".format(len(HTML))
        conn.send(hdr.encode())
        conn.send(HTML)

    elif path.startswith("/cmd") and method == "GET":
        idx = path.find("d=")
        if idx >= 0:
            cmd = path[idx+2:].split("&")[0].split(" ")[0]
            if cmd in VALID_CMDS:
                current_cmd = cmd
        last_cmd_ms = time.ticks_ms()
        conn.send(b"HTTP/1.0 200 OK\r\nContent-Length:2\r\n\r\nOK")

    elif path == "/status" and method == "GET":
        body = json.dumps({"dist": current_dist, "blocked": is_blocked})
        hdr = "HTTP/1.0 200 OK\r\nContent-Type:application/json\r\nContent-Length:{}\r\n\r\n".format(len(body))
        conn.send(hdr.encode())
        conn.send(body.encode())

    elif path == "/wifi" and method == "POST":
        bstart = data.find(b"\r\n\r\n")
        body = data[bstart + 4:] if bstart >= 0 else b""
        try:
            d = json.loads(body)
            save_wifi(d["ssid"], d["password"])
            ip = connect_wifi(d["ssid"], d["password"])
            if ip:
                resp = json.dumps({"status": "connected", "ip": ip})
                current_ip = ip
                ap_mode = False
            else:
                resp = json.dumps({"status": "failed"})
        except Exception:
            resp = json.dumps({"error": "bad request"})
        hdr = "HTTP/1.0 200 OK\r\nContent-Type:application/json\r\nContent-Length:{}\r\n\r\n".format(len(resp))
        conn.send(hdr.encode())
        conn.send(resp.encode())

    else:
        conn.send(b"HTTP/1.0 404 Not Found\r\nContent-Length:9\r\n\r\nNot Found")

    conn.close()


# Line Following (disabled, see line_follower_backup.py)
#
# def calibrate():
#     ...
#
# def get_line_position():
#     ...


# RGB Sensor (APDS9960, not wired)
# Wire to I2C0 (SDA=GP12, SCL=GP13), address 0x39.
#
# from APDS9960LITE import APDS9960LITE
# rgb_sensor = APDS9960LITE(i2c)
# rgb_sensor.als.enableSensor()
#
# def read_rgb():
#     return (rgb_sensor.als.redLightLevel,
#             rgb_sensor.als.greenLightLevel,
#             rgb_sensor.als.blueLightLevel)


# Startup

print("RC Car starting...")

if has_oled:
    oled.fill(0)
    oled.text("RC CAR", 40, 10)
    oled.text("Connecting...", 12, 30)
    oled.show()

ap_mode = True
current_ip = "?"

saved = load_wifi()
if saved:
    ip = connect_wifi(saved["ssid"], saved["password"])
    if ip:
        current_ip = ip
        ap_mode = False

if ap_mode:
    current_ip = start_ap()

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
srv.listen(2)
srv.settimeout(0)

print("Server: http://{}".format(current_ip))

if has_oled:
    oled.fill(0)
    oled.text("RC CAR READY", 8, 0)
    if ap_mode:
        oled.text("WiFi:", 0, 16)
        oled.text(AP_SSID, 0, 26)
        oled.text("Pass:" + AP_PASS, 0, 36)
    oled.text(current_ip, 0, 48)
    oled.show()
    time.sleep(4)


# Main Loop

loop_count = 0
dist_tick = 0

try:
    while True:
        try:
            conn, addr = srv.accept()
            handle_http(conn)
        except OSError:
            pass

        if time.ticks_diff(time.ticks_ms(), last_cmd_ms) > CMD_TIMEOUT_MS:
            if current_cmd != "S":
                current_cmd = "S"

        dist_tick += 1
        if dist_tick >= 20:
            dist_tick = 0
            current_dist = read_distance_mm()

        is_blocked = apply_drive(current_cmd, current_dist)
        update_wheel_speeds()
        ir_l, ir_c, ir_r = read_ir()

        try:
            update_oled(current_cmd, current_dist, is_blocked, ir_l, ir_c, ir_r)
        except Exception:
            pass

        loop_count += 1
        if loop_count % 500 == 0:
            print("cmd:{} dist:{}mm bl:{} Lspd:{} Rspd:{}".format(
                current_cmd, current_dist, is_blocked, left_tps, right_tps))

        time.sleep(0.005)

except Exception as e:
    left_motor.stop()
    right_motor.stop()
    servo.deinit()
    print("CRASH:", e)
