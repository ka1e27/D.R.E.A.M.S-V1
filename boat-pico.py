from machine import Pin, PWM, UART, I2C, ADC
import uasyncio as asyncio
import math
import time
import dht

# HARDWARE
radio = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))
gps_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
try:    i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=100000)
except: i2c = None

ML_PIN = 6; MR_PIN = 7; MW_PIN = 14
try:
    ml = PWM(Pin(ML_PIN)); ml.freq(50)
    mr = PWM(Pin(MR_PIN)); mr.freq(50)
    mw = PWM(Pin(MW_PIN)); mw.freq(50)
except:
    ml = None; mr = None; mw = None

try:    batt_adc = ADC(26)
except: batt_adc = None
try:    mt_l_adc = ADC(27); mt_r_adc = ADC(28)
except: mt_l_adc = None; mt_r_adc = None
try:    dht_s = dht.DHT11(Pin(16))
except: dht_s = None
try:    hull_l = Pin(17, Pin.IN, Pin.PULL_UP); hull_r = Pin(18, Pin.IN, Pin.PULL_UP)
except: hull_l = None; hull_r = None

# CONSTANTS
STOP_PWM = 4915; MIN_PWM = 3277; MAX_PWM = 6553
FWD_RANGE = MAX_PWM - STOP_PWM
REV_RANGE = STOP_PWM - MIN_PWM

QMC_ADDR = 0x2C
X_OFF = 644; Y_OFF = -51; HDG_OFF = 130

PULLEY_L = 18.0 / 27.0
PULLEY_R = 15.0 / 30.0
COMP_L = PULLEY_R / PULLEY_L
COMP_R = 1.0
TRIM = 0.0

ARRIVE_M = 5; HOLD_DRIFT_M = 8; HDG_TOL = 10
NAV_SPD = 40; MANUAL_SPD = 50; WINCH_SPD = 50
TURN_MIN = 15; MAX_HDG_ERR = 90; GPS_STALE_MS = 3000
GPS_TRACK_MIN_DIST = 3.0

B_RATIO = 15.0 / 115.0; INV_B_RATIO = 1.0 / B_RATIO
B_FULL = 16.8; B_EMPTY = 13.2; B_WARN = 14.0; B_CRIT = 13.6
B_RANGE = B_FULL - B_EMPTY

TH_NOM = 10000; TH_B = 3950; TH_R = 10000
INV_T_NOM = 1.0 / 298.15; INV_B_COEFF = 1.0 / TH_B
MT_WARN = 70; MT_CRIT = 85

W_DESCENT_RATE = 0.10
W_DN_SPD = 50; W_UP_SPD = -50
W_UP_MARGIN = 1.5

PROFILE_TIME_INTERVAL = 10.0
BOTTOM_CONFIRM_NEEDED = 3

SAMPLE_INTERVAL_S = 5.0
SAMPLE_MAX_TIMEOUT = 300
BOTTOM_LOST_CONFIRM = 3
MIN_BOTTOM_SAMPLES = 2

WP_DATA = 0; WP_PATH = 1

PH_TRANSIT = 0; PH_WINCH_DN = 1; PH_SAMPLE = 2
PH_WINCH_UP = 3; PH_PROBE = 4; PH_HOLD = 5
PHASE_NAME = ("TRANSIT", "WINCH_DN", "SAMPLE", "WINCH_UP", "PROBE", "HOLD")

FAIL_TH = 10; TIMEOUT_MS = 5000
TX_MAX_Q = 20; TX_GAP_MS = 80
TX_BURST_MS = 1500; TX_LISTEN_MS = 2500; TX_DRAIN_MS = 250
POLL_INTERVAL = 5; POD_TIMEOUT_MS = 10000
RADIO_LOST_MS = 60000

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
EARTH_R = 6371000.0

# STATE
gps_lat = 0.0; gps_lon = 0.0; gps_spd = 0.0
gps_ok = False; gps_last_t = 0
heading = 0.0

gps_track_hdg = 0.0; gps_track_valid = False
gps_prev_lat = 0.0; gps_prev_lon = 0.0

ml_spd = 0; mr_spd = 0; w_spd = 0
throttle_mult = 1.0

tx_hold_until = 0; tx_cycle_start = 0
tx_in_burst = True; tx_listening = False

batt_v = 0.0; batt_pct = 0; batt_warned = False
air_t = 0; air_h = 0
hull_lw = False; hull_rw = False
hull_sent_l = False; hull_sent_r = False
mt_l = 0.0; mt_r = 0.0; mt_throttled = False

water_t = -999.0; water_ec = 0.0; water_light = 0
pod_ok = False; pod_last_t = 0; bottom_contact = False

nav_on = False; tgt_lat = None; tgt_lon = None
rc_over = False

mission_wps = []
mis_idx = 0; mis_run = False
mis_phase = PH_TRANSIT; phase_tmr = 0
winch_dn_start = 0; winch_dn_elapsed = 0.0; est_depth = 0.0
last_profile_time = 0.0; profile_data = []
bottom_hit_count = 0; depth_confirmed = False; probe_tmr = 0
sample_count = 0; last_sample_time = 0.0
sample_start_time = 0; bottom_lost_count = 0

hold_lat = 0.0; hold_lon = 0.0; holding = False
hold_cooldown = 0

eng_dist = 0.0; eng_v_start = 0.0
eng_last_lat = 0.0; eng_last_lon = 0.0
eng_started = False; eng_v_per_m = 0.0

safety_overrides = set()
CRITICAL_OVERRIDES = {"battery", "hull"}

hlth = {}
for _n in ("compass", "gps", "battery", "dht", "mtemp", "pod"):
    hlth[_n] = [False, 0, 0]

radio_ok = True; radio_last_rx = 0; radio_lost = False
tx_queue = []; tx_last = 0
rx_buf = bytearray()
_u1_buf = bytearray(128); _u1_pos = 0

# RADIO FRAMING
def _cs(s):
    c = 0
    for ch in s: c ^= ord(ch)
    return c

def tx(msg):
    if len(tx_queue) >= TX_MAX_Q: tx_queue.pop(0)
    tx_queue.append("$%s*%d\n" % (msg, _cs(msg)))

def tx_now(msg):
    global tx_hold_until
    framed = "$%s*%d\n" % (msg, _cs(msg))
    tx_queue.insert(0, framed)
    if len(tx_queue) > TX_MAX_Q: tx_queue.pop()
    tx_hold_until = time.ticks_ms() + 300

def rx_frames(raw):
    global rx_buf
    rx_buf.extend(raw)
    if len(rx_buf) > 1024: rx_buf = rx_buf[-512:]
    out = []
    while True:
        si = -1
        for i in range(len(rx_buf)):
            if rx_buf[i] == 0x24: si = i; break
        if si < 0: rx_buf = bytearray(); break
        if si > 0: rx_buf = rx_buf[si:]
        ei = -1
        for i in range(1, len(rx_buf)):
            if rx_buf[i] == 0x0a: ei = i; break
        if ei < 0: break
        frame = rx_buf[1:ei]
        rx_buf = rx_buf[ei + 1:]
        try:
            s = frame.decode('utf-8', 'ignore').strip()
            if '*' in s:
                p, c = s.rsplit('*', 1)
                if _cs(p) == int(c): out.append(p)
        except: pass
    return out

# SENSOR HEALTH
def s_ok(name):
    h = hlth[name]
    if not h[0]: return False
    if h[2] > 0 and time.ticks_diff(time.ticks_ms(), h[2]) > TIMEOUT_MS:
        h[0] = False; tx("STATUS,SENSOR_FAULT,%s" % name)
        return False
    return True

def s_pass(name):
    h = hlth[name]; was_bad = not h[0]
    h[0] = True; h[1] = 0; h[2] = time.ticks_ms()
    if was_bad: tx("STATUS,SENSOR_OK,%s" % name)

def s_fail(name):
    h = hlth[name]; h[1] += 1
    if h[1] >= FAIL_TH and h[0]:
        h[0] = False; tx("STATUS,SENSOR_FAULT,%s" % name)

# MOTOR CONTROL
def set_m(motor, spd):
    if motor is None: return
    if spd > 100: spd = 100
    elif spd < -100: spd = -100
    if spd >= 0: duty = STOP_PWM + FWD_RANGE * spd // 100
    else:        duty = STOP_PWM + REV_RANGE * spd // 100
    try: motor.duty_u16(duty)
    except: pass

def motors_update():
    if not rc_over:
        eff_ml = int(-ml_spd * throttle_mult * (COMP_L + TRIM))
        eff_mr = int(mr_spd * throttle_mult * (COMP_R - TRIM))
        set_m(ml, eff_ml); set_m(mr, eff_mr); set_m(mw, w_spd)

def rc_on():
    global rc_over, ml, mr, mw, nav_on, ml_spd, mr_spd, w_spd, holding, mis_run
    if mis_run: abort_mis("RC_OVERRIDE")
    nav_on = False; holding = False; ml_spd = 0; mr_spd = 0; w_spd = 0
    try:
        if ml: ml.deinit()
        if mr: mr.deinit()
        if mw: mw.deinit()
    except: pass
    Pin(ML_PIN, Pin.IN); Pin(MR_PIN, Pin.IN); Pin(MW_PIN, Pin.IN)
    ml = None; mr = None; mw = None
    rc_over = True; tx("STATUS,RC_OVERRIDE_ON")

def rc_off():
    global rc_over, ml, mr, mw
    try:
        ml = PWM(Pin(ML_PIN)); mr = PWM(Pin(MR_PIN)); mw = PWM(Pin(MW_PIN))
        ml.freq(50); mr.freq(50); mw.freq(50)
        set_m(ml, 0); set_m(mr, 0); set_m(mw, 0)
    except Exception as e:
        tx("STATUS,ERR,rc_off,%s" % e)
    rc_over = False; tx("STATUS,RC_OVERRIDE_OFF")

# UART1 SHARED PARSER
def _process_line(line):
    global gps_lat, gps_lon, gps_spd, gps_ok, gps_last_t
    global water_t, water_ec, water_light, pod_ok, pod_last_t, bottom_contact
    if not line or line[0] != '$': return
    if line[1:4] == 'GPR' or line[1:4] == 'GNR':
        try:
            p = line.split(',')
            if len(p) >= 8 and p[2] == 'A':
                gps_ok = True; gps_last_t = time.ticks_ms()
                raw = float(p[3]); deg = int(raw / 100)
                gps_lat = deg + (raw - deg * 100) / 60.0
                if p[4] == 'S': gps_lat = -gps_lat
                raw = float(p[5]); deg = int(raw / 100)
                gps_lon = deg + (raw - deg * 100) / 60.0
                if p[6] == 'W': gps_lon = -gps_lon
                if p[7]: gps_spd = float(p[7])
                s_pass("gps")
            else: gps_ok = False
        except: s_fail("gps")
    elif line[1:4] == 'POD':
        try:
            if '*' not in line: return
            payload, cs_str = line[1:].rsplit('*', 1)
            cs_calc = 0
            for ch in payload: cs_calc ^= ord(ch)
            if cs_calc != int(cs_str): return
            parts = payload.split(',')
            if len(parts) >= 5:
                water_t = float(parts[1]); water_ec = float(parts[2])
                water_light = int(parts[3])
                bottom_contact = parts[4] == '1'
                pod_ok = True; pod_last_t = time.ticks_ms()
                s_pass("pod")
        except: s_fail("pod")

def read_uart1():
    global _u1_pos
    while gps_uart.any():
        chunk = gps_uart.read()
        if not chunk: break
        for b in chunk:
            if b == 0x0a:
                if _u1_pos > 0:
                    try:
                        line = _u1_buf[:_u1_pos].decode('utf-8', 'ignore').strip()
                        _process_line(line)
                    except: pass
                _u1_pos = 0
            elif b == 0x24:
                _u1_pos = 0; _u1_buf[0] = b; _u1_pos = 1
            elif _u1_pos < 128:
                _u1_buf[_u1_pos] = b; _u1_pos += 1

def poll_pod():
    try: gps_uart.write("$POLL*%d\n" % _cs("POLL"))
    except: s_fail("pod")

# SENSOR READS
def init_compass():
    if i2c is None: return False
    try:
        i2c.writeto_mem(QMC_ADDR, 0x09, b'\x3d')
        i2c.writeto_mem(QMC_ADDR, 0x0a, b'\x01')
        s_pass("compass"); return True
    except: s_fail("compass"); return False

def read_compass():
    global heading
    if i2c is None: s_fail("compass"); return
    try:
        d = i2c.readfrom_mem(QMC_ADDR, 0x01, 6)
        x = (d[0] | (d[1] << 8)); y = (d[2] | (d[3] << 8))
        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        x -= X_OFF; y -= Y_OFF
        h = math.atan2(-y, x) * RAD_TO_DEG
        if h < 0: h += 360.0
        heading = (h - HDG_OFF) % 360.0
        s_pass("compass")
    except: s_fail("compass")

def read_batt():
    global batt_v, batt_pct
    if batt_adc is None: s_fail("battery"); return
    try:
        total = 0
        for _ in range(16): total += batt_adc.read_u16()
        batt_v = (total / 16.0 / 65535.0 * 3.3) * INV_B_RATIO
        batt_pct = max(0, min(100, int((batt_v - B_EMPTY) / B_RANGE * 100)))
        if batt_v < 3.0: s_fail("battery")
        else: s_pass("battery")
    except: s_fail("battery")

def read_dht():
    global air_t, air_h
    if dht_s is None: s_fail("dht"); return
    try:
        dht_s.measure(); air_t = dht_s.temperature(); air_h = dht_s.humidity()
        s_pass("dht")
    except: s_fail("dht")

def read_hull():
    global hull_lw, hull_rw
    if hull_l is not None: hull_lw = hull_l.value() == 0
    if hull_r is not None: hull_rw = hull_r.value() == 0

def _ntc(adc):
    total = 0
    for _ in range(8): total += adc.read_u16()
    raw = total >> 3
    if raw < 100 or raw > 65400: return None
    resistance = TH_R * raw / (65535.0 - raw)
    if resistance <= 0: return None
    steinhart = math.log(resistance / TH_NOM) * INV_B_COEFF + INV_T_NOM
    if steinhart <= 0: return None
    return 1.0 / steinhart - 273.15

def read_mtemp():
    global mt_l, mt_r
    if mt_l_adc is None: s_fail("mtemp"); return
    try:
        left = _ntc(mt_l_adc)
        right = _ntc(mt_r_adc) if mt_r_adc is not None else None
        if left is not None: mt_l = left
        if right is not None: mt_r = right
        if left is not None or right is not None: s_pass("mtemp")
        else: s_fail("mtemp")
    except: s_fail("mtemp")

# NAV HELPERS
def hav_dist(lat1, lon1, lat2, lon2):
    r1 = lat1 * DEG_TO_RAD; r2 = lat2 * DEG_TO_RAD
    dlat = (lat2 - lat1) * DEG_TO_RAD; dlon = (lon2 - lon1) * DEG_TO_RAD
    a = math.sin(dlat * 0.5) ** 2 + math.cos(r1) * math.cos(r2) * math.sin(dlon * 0.5) ** 2
    return EARTH_R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

def calc_bearing(lat1, lon1, lat2, lon2):
    r1 = lat1 * DEG_TO_RAD; r2 = lat2 * DEG_TO_RAD
    dlon = (lon2 - lon1) * DEG_TO_RAD
    x = math.sin(dlon) * math.cos(r2)
    y = math.cos(r1) * math.sin(r2) - math.sin(r1) * math.cos(r2) * math.cos(dlon)
    return math.atan2(x, y) * RAD_TO_DEG % 360.0

def hdg_diff(current, target):
    d = target - current
    if d > 180: d -= 360
    elif d < -180: d += 360
    return d

def gps_stale():
    if gps_last_t == 0: return True
    return time.ticks_diff(time.ticks_ms(), gps_last_t) > GPS_STALE_MS

# STATION KEEPING
def start_hold(lat, lon):
    global hold_lat, hold_lon, holding, tgt_lat, tgt_lon, nav_on
    hold_lat = lat; hold_lon = lon; holding = True
    tgt_lat = lat; tgt_lon = lon; nav_on = False

def stop_hold():
    global holding
    holding = False

# MISSION HELPERS
def safe_stop(reason):
    global ml_spd, mr_spd, w_spd, nav_on, holding, mis_run
    if mis_run: abort_mis(reason)
    nav_on = False; holding = False; ml_spd = 0; mr_spd = 0; w_spd = 0
    tx("STATUS,SAFE_STOP,%s" % reason)

def start_mis():
    global mis_run, mis_idx, mis_phase, est_depth
    global eng_dist, eng_v_start, eng_last_lat, eng_last_lon
    global eng_started, eng_v_per_m
    if not mission_wps: tx_now("NAK,MISSION_START,NO_WP"); return
    if not s_ok("gps"): tx_now("NAK,MISSION_START,NO_GPS"); return
    mis_idx = 0; mis_run = True; mis_phase = PH_TRANSIT; est_depth = 0.0
    eng_dist = 0.0; eng_v_start = batt_v
    eng_last_lat = gps_lat; eng_last_lon = gps_lon
    eng_started = True; eng_v_per_m = 0.0
    set_transit(0); tx_now("ACK,MISSION_START,%d" % len(mission_wps))

def set_transit(idx):
    global tgt_lat, tgt_lon, nav_on, holding
    wp = mission_wps[idx]
    tgt_lat = wp[0]; tgt_lon = wp[1]; nav_on = True; holding = False
    tx("STATUS,TRANSIT,%d,%.6f,%.6f" % (idx, tgt_lat, tgt_lon))

def abort_mis(reason="ABORTED"):
    global mis_run, nav_on, ml_spd, mr_spd, w_spd, holding, est_depth
    mis_run = False; nav_on = False; holding = False
    ml_spd = 0; mr_spd = 0; w_spd = 0; est_depth = 0.0
    tx("STATUS,MISSION_%s" % reason)

def next_wp():
    global mis_idx, mis_phase
    stop_hold(); mis_idx += 1
    if mis_idx >= len(mission_wps):
        tx("STATUS,MISSION_COMPLETE,%d" % len(mission_wps))
        abort_mis("COMPLETE")
    else:
        mis_phase = PH_TRANSIT; set_transit(mis_idx)

# ASYNC TASKS
async def task_radio_tx():
    global tx_last, radio_ok, tx_cycle_start, tx_in_burst, tx_listening
    tx_cycle_start = time.ticks_ms()
    tx_in_burst = True; tx_listening = False
    while True:
        try:
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, tx_cycle_start)
            if tx_in_burst:
                tx_listening = False
                if elapsed >= TX_BURST_MS:
                    await asyncio.sleep_ms(TX_DRAIN_MS)
                    tx_in_burst = False; tx_listening = True
                    tx_cycle_start = time.ticks_ms()
                elif tx_queue:
                    if time.ticks_diff(now, tx_last) >= TX_GAP_MS:
                        if time.ticks_diff(now, tx_hold_until) >= 0 or len(tx_queue) <= 2:
                            try:
                                radio.write(tx_queue.pop(0)); radio_ok = True
                            except: radio_ok = False
                            tx_last = now
                else:
                    await asyncio.sleep_ms(TX_DRAIN_MS)
                    tx_in_burst = False; tx_listening = True
                    tx_cycle_start = time.ticks_ms()
            else:
                tx_listening = True
                if elapsed >= TX_LISTEN_MS:
                    tx_in_burst = True; tx_listening = False
                    tx_cycle_start = time.ticks_ms()
        except: pass
        await asyncio.sleep(0.02)

async def task_fast_sensors():
    poll_ctr = 0
    while True:
        try:
            read_uart1(); read_compass(); read_batt()
            poll_ctr += 1
            if poll_ctr >= POLL_INTERVAL: poll_ctr = 0; poll_pod()
        except Exception as e: tx("STATUS,ERR,fast,%s" % e)
        await asyncio.sleep(0.1)

async def task_slow_sensors():
    while True:
        try: read_dht(); read_hull(); read_mtemp()
        except Exception as e: tx("STATUS,ERR,slow,%s" % e)
        await asyncio.sleep(3)

async def task_motors():
    while True:
        try: motors_update()
        except: pass
        await asyncio.sleep(0.1)

async def task_telemetry():
    global pod_ok
    _nav_on = nav_on
    _mis_run = mis_run
    _holding = holding
    while True:
        try:
            _nav_on = nav_on
            _mis_run = mis_run
            _holding = holding
            if pod_ok and pod_last_t > 0:
                if time.ticks_diff(time.ticks_ms(), pod_last_t) > POD_TIMEOUT_MS:
                    pod_ok = False
            if rc_over:       st = "RC"
            elif _mis_run:    st = "M%d/%d" % (mis_idx, len(mission_wps))
            elif _nav_on:     st = "NAV"
            elif _holding:    st = "HOLD"
            elif gps_ok:      st = "OK"
            else:             st = "NO_FIX"
            hl = ""
            if hull_lw: hl += "L!"
            if hull_rw: hl += "R!"
            if not hl: hl = "DRY"
            qf = ""
            qf += "1" if s_ok("gps") else "0"
            qf += "1" if s_ok("compass") else "0"
            qf += "1" if s_ok("battery") else "0"
            qf += "1" if s_ok("dht") else "0"
            qf += "1" if s_ok("mtemp") else "0"
            qf += "1" if pod_ok else "0"
            if gps_track_valid and gps_spd > 0.3:
                disp_hdg = gps_track_hdg
            else:
                disp_hdg = heading
            m1 = "D1,%.6f,%.6f,%.1f,%.1f,%s,%.1f,%d,%s,Q%s" % (
                gps_lat, gps_lon, disp_hdg, gps_spd, st,
                batt_v, batt_pct, hl, qf)
            if _nav_on and tgt_lat is not None:
                m1 += ",%.1f" % hav_dist(gps_lat, gps_lon, tgt_lat, tgt_lon)
            if _mis_run:
                m1 += ",%s,%.1f" % (PHASE_NAME[mis_phase], est_depth)
            tx(m1)
            if eng_started and eng_v_per_m > 0:
                tx("D3,%.4f,%.1f,%.2f" % (eng_v_per_m, eng_dist, eng_v_per_m * 1000.0))
            btm = "BTM" if bottom_contact else ""
            m2 = "D2,%d,%d,%.0f,%.0f,%.1f,%.1f,%d,%s" % (
                air_t, air_h, mt_l, mt_r, water_t, water_ec, water_light, btm)
            tx(m2)
        except Exception as e: tx("STATUS,ERR,telem,%s" % e)
        await asyncio.sleep(5)

async def task_navigate():
    global ml_spd, mr_spd, nav_on, tgt_lat, tgt_lon, hold_cooldown
    global eng_dist, eng_last_lat, eng_last_lon, eng_v_per_m, eng_started
    global gps_track_hdg, gps_track_valid, gps_prev_lat, gps_prev_lon
    global holding, mis_run
    while True:
        try:
            if gps_ok and not gps_stale():
                if gps_prev_lat == 0.0 and gps_prev_lon == 0.0:
                    gps_prev_lat = gps_lat; gps_prev_lon = gps_lon
                else:
                    trk_seg = hav_dist(gps_prev_lat, gps_prev_lon, gps_lat, gps_lon)
                    if trk_seg >= GPS_TRACK_MIN_DIST:
                        gps_track_hdg = calc_bearing(gps_prev_lat, gps_prev_lon, gps_lat, gps_lon)
                        gps_track_valid = True
                        gps_prev_lat = gps_lat; gps_prev_lon = gps_lon

            if eng_started and gps_ok and not gps_stale():
                if nav_on or mis_run:
                    seg = hav_dist(eng_last_lat, eng_last_lon, gps_lat, gps_lon)
                    if seg > 1.0:
                        eng_dist += seg
                        eng_last_lat = gps_lat; eng_last_lon = gps_lon
                        if eng_dist > 10.0:
                            v_used = eng_v_start - batt_v
                            if v_used > 0: eng_v_per_m = v_used / eng_dist

            if holding and not nav_on and gps_ok and not gps_stale():
                now = time.ticks_ms()
                if time.ticks_diff(now, hold_cooldown) > 3000:
                    drift = hav_dist(gps_lat, gps_lon, hold_lat, hold_lon)
                    if drift > HOLD_DRIFT_M:
                        tgt_lat = hold_lat; tgt_lon = hold_lon; nav_on = True
                        tx("STATUS,HOLD_REACQUIRE,%.1f" % drift)

            if nav_on and tgt_lat is not None:
                if not s_ok("gps") or gps_stale() or not s_ok("compass"):
                    ml_spd = 0; mr_spd = 0
                    await asyncio.sleep(0.2); continue
                dist = hav_dist(gps_lat, gps_lon, tgt_lat, tgt_lon)
                if dist < ARRIVE_M:
                    ml_spd = 0; mr_spd = 0
                    if not mis_run and not holding:
                        nav_on = False; tgt_lat = None; tgt_lon = None
                        tx("STATUS,ARRIVED")
                    else:
                        nav_on = False; hold_cooldown = time.ticks_ms()
                else:
                    req = calc_bearing(gps_lat, gps_lon, tgt_lat, tgt_lon)
                    if gps_track_valid and gps_spd > 0.3:
                        cur_hdg = gps_track_hdg
                    else:
                        cur_hdg = heading
                    err = hdg_diff(cur_hdg, req)
                    if abs(err) < HDG_TOL: blend = 0.0
                    else: blend = min((abs(err) - HDG_TOL) / (MAX_HDG_ERR - HDG_TOL), 1.0)
                    if err >= 0:
                        ml_spd = NAV_SPD
                        mr_spd = int(NAV_SPD * (1.0 - blend))
                    else:
                        ml_spd = int(NAV_SPD * (1.0 - blend))
                        mr_spd = NAV_SPD
        except Exception as e:
            ml_spd = 0; mr_spd = 0
            tx("STATUS,ERR,nav,%s" % e)
        await asyncio.sleep(0.2)

async def task_mission():
    global mis_phase, phase_tmr, mis_idx, w_spd
    global winch_dn_start, winch_dn_elapsed, est_depth
    global last_profile_time, profile_data
    global bottom_hit_count, depth_confirmed, probe_tmr
    global sample_count, last_sample_time, sample_start_time, bottom_lost_count
    while True:
        if not mis_run: await asyncio.sleep(0.5); continue
        try:
            wp = mission_wps[mis_idx]
            max_timeout = wp[2] if len(wp) > 2 else SAMPLE_MAX_TIMEOUT
            wp_type = wp[3] if len(wp) > 3 else WP_DATA

            if mis_phase == PH_TRANSIT:
                if not nav_on:
                    if wp_type == WP_PATH:
                        tx("STATUS,PATH_PASS,%d,%.6f,%.6f" % (mis_idx, wp[0], wp[1]))
                        next_wp(); continue
                    start_hold(wp[0], wp[1])
                    mis_phase = PH_WINCH_DN; phase_tmr = time.ticks_ms()
                    winch_dn_start = time.ticks_ms()
                    winch_dn_elapsed = 0.0; est_depth = 0.0
                    last_profile_time = 0.0; profile_data = []
                    bottom_hit_count = 0; depth_confirmed = False
                    w_spd = W_DN_SPD
                    tx("STATUS,WINCH_DOWN,%d" % mis_idx)

            elif mis_phase == PH_WINCH_DN:
                now = time.ticks_ms()
                winch_dn_elapsed = time.ticks_diff(now, winch_dn_start) / 1000.0
                est_depth = winch_dn_elapsed * W_DESCENT_RATE
                if winch_dn_elapsed - last_profile_time >= PROFILE_TIME_INTERVAL:
                    last_profile_time = winch_dn_elapsed
                    profile_data.append((est_depth, water_t, water_ec, water_light))
                    tx("STATUS,PROFILE,%d,%.2f,%.1f,%.1f,%d" % (
                        mis_idx, est_depth, water_t, water_ec, water_light))
                if bottom_contact:
                    bottom_hit_count += 1
                    if bottom_hit_count >= BOTTOM_CONFIRM_NEEDED:
                        w_spd = 0; depth_confirmed = True
                        profile_data.append((est_depth, water_t, water_ec, water_light))
                        tx("STATUS,BOTTOM_CONFIRMED,%d,%.2f,%.1fs" % (
                            mis_idx, est_depth, winch_dn_elapsed))
                        mis_phase = PH_SAMPLE
                        sample_start_time = time.ticks_ms()
                        sample_count = 0; last_sample_time = 0.0; bottom_lost_count = 0
                        tx("STATUS,SAMPLING,%d,CONTACT_MODE,max%ds" % (mis_idx, max_timeout))
                    else:
                        w_spd = W_UP_SPD; mis_phase = PH_PROBE
                        probe_tmr = time.ticks_ms()
                        tx("STATUS,BOTTOM_PROBE,%d,%d,%.2f" % (
                            mis_idx, bottom_hit_count, est_depth))
                else: bottom_hit_count = 0

            elif mis_phase == PH_PROBE:
                elapsed = time.ticks_diff(time.ticks_ms(), probe_tmr)
                if elapsed < 500:    w_spd = W_UP_SPD
                elif elapsed < 800:  w_spd = 0
                else:
                    w_spd = W_DN_SPD; mis_phase = PH_WINCH_DN
                    phase_tmr = time.ticks_ms()

            elif mis_phase == PH_SAMPLE:
                elapsed_s = time.ticks_diff(time.ticks_ms(), sample_start_time) / 1000.0
                if elapsed_s - last_sample_time >= SAMPLE_INTERVAL_S:
                    last_sample_time = elapsed_s; sample_count += 1
                    profile_data.append((est_depth, water_t, water_ec, water_light))
                    tx("STATUS,SAMPLE_DATA,%d,%d,%.2f,%.1f,%.1f,%d,%s" % (
                        mis_idx, sample_count, est_depth,
                        water_t, water_ec, water_light,
                        "BTM" if bottom_contact else "NO_BTM"))
                if not bottom_contact:
                    bottom_lost_count += 1
                    if bottom_lost_count >= BOTTOM_LOST_CONFIRM and sample_count >= MIN_BOTTOM_SAMPLES:
                        tx("STATUS,CONTACT_LOST,%d,%d_samples,%.1fs" % (
                            mis_idx, sample_count, elapsed_s))
                        sent = 0
                        while profile_data and sent < 4:
                            d, t, e, l = profile_data.pop(0)
                            tx("STATUS,PDATA,%d,%d,%.2f,%.1f,%.1f,%d" % (mis_idx, sent, d, t, e, l))
                            sent += 1
                        profile_data = []
                        mis_phase = PH_WINCH_UP; phase_tmr = time.ticks_ms()
                        w_spd = W_UP_SPD
                        tx("STATUS,WINCH_UP,%d,%.1fs,%d_samples" % (
                            mis_idx, winch_dn_elapsed, sample_count))
                else: bottom_lost_count = 0
                if elapsed_s >= max_timeout:
                    tx("STATUS,SAMPLE_TIMEOUT,%d,%d_samples,%.0fs" % (
                        mis_idx, sample_count, elapsed_s))
                    sent = 0
                    while profile_data and sent < 4:
                        d, t, e, l = profile_data.pop(0)
                        tx("STATUS,PDATA,%d,%d,%.2f,%.1f,%.1f,%d" % (mis_idx, sent, d, t, e, l))
                        sent += 1
                    profile_data = []
                    mis_phase = PH_WINCH_UP; phase_tmr = time.ticks_ms()
                    w_spd = W_UP_SPD
                    tx("STATUS,WINCH_UP,%d,%.1fs,%d_samples" % (
                        mis_idx, winch_dn_elapsed, sample_count))

            elif mis_phase == PH_WINCH_UP:
                raise_time = winch_dn_elapsed * W_UP_MARGIN
                elapsed_s = time.ticks_diff(time.ticks_ms(), phase_tmr) / 1000.0
                if elapsed_s >= raise_time: w_spd = 0; next_wp()

        except Exception as e:
            w_spd = 0; tx("STATUS,ERR,mission,%s" % e)
        await asyncio.sleep(0.5)

async def task_watchdogs():
    global batt_warned, hull_sent_l, hull_sent_r, mt_throttled, throttle_mult
    _nav_on = nav_on
    _mis_run = mis_run
    while True:
        try:
            _nav_on = nav_on
            _mis_run = mis_run
            for name in hlth: s_ok(name)
            if (_nav_on or _mis_run) and not s_ok("compass"):
                if "compass" not in safety_overrides: safe_stop("COMPASS_FAULT")
                else: tx("STATUS,WARN_OVERRIDE,compass")
            if (_nav_on or _mis_run) and not s_ok("gps"):
                if "gps" not in safety_overrides: safe_stop("GPS_FAULT")
                else: tx("STATUS,WARN_OVERRIDE,gps")
            if batt_v > 3.0:
                if batt_v <= B_CRIT:
                    if "battery" not in safety_overrides: safe_stop("BATT_CRITICAL")
                    tx("STATUS,BATT_CRITICAL,%.2f,%d" % (batt_v, batt_pct))
                elif batt_v <= B_WARN and not batt_warned:
                    batt_warned = True
                    tx("STATUS,BATT_LOW,%.2f,%d" % (batt_v, batt_pct))
                elif batt_v > B_WARN: batt_warned = False
            if hull_lw and not hull_sent_l: hull_sent_l = True; tx("STATUS,HULL_LEAK,LEFT")
            elif not hull_lw and hull_sent_l: hull_sent_l = False
            if hull_rw and not hull_sent_r: hull_sent_r = True; tx("STATUS,HULL_LEAK,RIGHT")
            elif not hull_rw and hull_sent_r: hull_sent_r = False
            if hull_lw and hull_rw:
                if "hull" not in safety_overrides: safe_stop("HULL_LEAK_BOTH")
                else: tx("STATUS,WARN_OVERRIDE,hull")
            mx = max(mt_l, mt_r); side = "LEFT" if mt_l >= mt_r else "RIGHT"
            if mx >= MT_CRIT and not rc_over:
                if "motor" not in safety_overrides:
                    safe_stop("MOTOR_OVERHEAT_%s_%.0fC" % (side, mx))
                else: tx("STATUS,WARN_OVERRIDE,motor,%.0f" % mx)
            elif mx >= MT_WARN:
                if not mt_throttled: mt_throttled = True; tx("STATUS,MOTOR_HOT,%s,%.0f" % (side, mx))
                if not rc_over and (_nav_on or _mis_run):
                    if "motor" not in safety_overrides: throttle_mult = 0.5
            elif mx < MT_WARN - 10 and mt_throttled:
                mt_throttled = False; throttle_mult = 1.0; tx("STATUS,MOTOR_COOL,%.0f" % mx)
            faults = [n for n in hlth if not hlth[n][0]]
            if not radio_ok: faults.append("radio")
            if faults: tx("STATUS,HEALTH,FAULT,%s" % '+'.join(faults))
            if safety_overrides: tx("STATUS,SAFETY_OVERRIDES,%s" % '+'.join(safety_overrides))
        except Exception as e: tx("STATUS,ERR,watch,%s" % e)
        await asyncio.sleep(3)

async def task_commands():
    global ml_spd, mr_spd, w_spd, nav_on, tgt_lat, tgt_lon
    global mission_wps, holding, HDG_OFF, TRIM, radio_last_rx
    global NAV_SPD, MANUAL_SPD, WINCH_SPD, W_DN_SPD, W_UP_SPD
    global mis_run, mis_phase, sample_count, est_depth, winch_dn_elapsed, profile_data
    while True:
        try:
            if radio.any():
                try: raw = radio.read()
                except: await asyncio.sleep(0.05); continue
                if not raw: await asyncio.sleep(0.05); continue
                for msg in rx_frames(raw):
                    radio_last_rx = time.ticks_ms()
                    if msg[0] == 'A' or msg[0] == 'N':
                        if msg[:3] in ("ACK", "NAK"): continue
                    p = msg.split(","); c = p[0]

                    if c == "PING": pass

                    elif c == "GOTO" and len(p) == 3:
                        if rc_over: tx_now("NAK,GOTO,RC_ACTIVE"); continue
                        try:
                            la = float(p[1]); lo = float(p[2])
                            if -90 <= la <= 90 and -180 <= lo <= 180:
                                if mis_run: abort_mis("ABORTED")
                                holding = False
                                tgt_lat = la; tgt_lon = lo; nav_on = True
                                tx_now("ACK,GOTO,%.6f,%.6f" % (la, lo))
                            else: tx_now("NAK,GOTO,RANGE")
                        except: tx_now("NAK,GOTO,PARSE")

                    elif c == "MISSION" and len(p) >= 2:
                        try:
                            n = int(p[1])
                            if len(p) == 2 + n * 4:
                                wps = []; ok = True
                                for i in range(n):
                                    b = 2 + i * 4
                                    la = float(p[b]); lo = float(p[b+1])
                                    st = int(p[b+2]); wt = int(p[b+3])
                                    if not (-90 <= la <= 90 and -180 <= lo <= 180):
                                        tx_now("NAK,MISSION,RANGE,WP%d" % i); ok = False; break
                                    wps.append((la, lo, st, wt))
                                if ok:
                                    if mis_run: abort_mis("REPLACED")
                                    mission_wps = wps
                                    data_n = sum(1 for w in wps if w[3] == WP_DATA)
                                    path_n = len(wps) - data_n
                                    tx_now("ACK,MISSION_LOADED,%d,%dD,%dP" % (n, data_n, path_n))
                            elif len(p) == 2 + n * 3:
                                wps = []; ok = True
                                for i in range(n):
                                    b = 2 + i * 3
                                    la = float(p[b]); lo = float(p[b+1]); st = int(p[b+2])
                                    if not (-90 <= la <= 90 and -180 <= lo <= 180):
                                        tx_now("NAK,MISSION,RANGE,WP%d" % i); ok = False; break
                                    wps.append((la, lo, st, WP_DATA))
                                if ok:
                                    if mis_run: abort_mis("REPLACED")
                                    mission_wps = wps
                                    tx_now("ACK,MISSION_LOADED,%d" % n)
                            else: tx_now("NAK,MISSION,LEN")
                        except: tx_now("NAK,MISSION,PARSE")

                    elif c == "MISSION_START":
                        if rc_over: tx_now("NAK,MISSION_START,RC_ACTIVE")
                        else: start_mis()

                    elif c == "MISSION_ABORT":
                        if mis_run: abort_mis("ABORTED")
                        tx_now("ACK,MISSION_ABORT")

                    elif c == "CANCEL_NAV":
                        if mis_run: abort_mis("ABORTED")
                        nav_on = False; holding = False
                        tgt_lat = None; tgt_lon = None; ml_spd = 0; mr_spd = 0
                        tx_now("ACK,CANCEL_NAV")

                    elif c in ("FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"):
                        if mis_run: abort_mis("ABORTED")
                        if rc_over: rc_off()
                        nav_on = False; holding = False
                        if c == "FORWARD":    ml_spd = MANUAL_SPD;  mr_spd = MANUAL_SPD
                        elif c == "BACKWARD": ml_spd = -MANUAL_SPD; mr_spd = -MANUAL_SPD
                        elif c == "LEFT":     ml_spd = -TURN_MIN;   mr_spd = MANUAL_SPD
                        elif c == "RIGHT":    ml_spd = MANUAL_SPD;  mr_spd = -TURN_MIN
                        else:                 ml_spd = 0;   mr_spd = 0
                        tx_now("ACK,%s" % c)

                    elif c in ("WINCH_DOWN", "WINCH_UP", "WINCH_STOP"):
                        if rc_over: tx_now("NAK,WINCH,RC"); continue
                        if c == "WINCH_UP" and mis_run:
                            skip_phase = PHASE_NAME[mis_phase] if mis_phase < len(PHASE_NAME) else "?"
                            sent = 0
                            while profile_data and sent < 4:
                                d, t, e, l = profile_data.pop(0)
                                tx("STATUS,PDATA,%d,%d,%.2f,%.1f,%.1f,%d" % (mis_idx, sent, d, t, e, l))
                                sent += 1
                            profile_data = []
                            tx("STATUS,SKIP_DROP,%d,%s,%d_samples,%.2fm,%.1fs" % (
                                mis_idx, skip_phase, sample_count, est_depth, winch_dn_elapsed))
                            mis_phase = PH_WINCH_UP; phase_tmr = time.ticks_ms()
                            w_spd = W_UP_SPD
                            tx_now("ACK,WINCH_UP,SKIP_DROP,%d" % mis_idx)
                        elif mis_run:
                            tx_now("NAK,WINCH,MISSION")
                        else:
                            if c == "WINCH_DOWN":  w_spd = W_DN_SPD
                            elif c == "WINCH_UP":  w_spd = W_UP_SPD
                            else:                  w_spd = 0
                            tx_now("ACK,%s,%d" % (c, abs(w_spd)))

                    elif c == "ALL_STOP":
                        if mis_run: abort_mis("ABORTED")
                        if rc_over: rc_off()
                        nav_on = False; holding = False
                        ml_spd = 0; mr_spd = 0; w_spd = 0
                        tx_now("ACK,ALL_STOP")

                    elif c == "ESTOP":
                        if mis_run: abort_mis("ABORTED")
                        if not rc_over: rc_on()
                        nav_on = False; holding = False
                        ml_spd = 0; mr_spd = 0; w_spd = 0
                        tx_now("ACK,ESTOP,RC_ON")

                    elif c == "RC_OVERRIDE":
                        if not rc_over: rc_on()
                        tx_now("ACK,RC_OVERRIDE")

                    elif c == "RC_RELEASE":
                        if rc_over: rc_off()
                        tx_now("ACK,RC_RELEASE")

                    elif c == "STATUS":
                        if rc_over: tx_now("ACK,STATUS,RC_OVERRIDE")
                        elif mis_run:
                            wp = mission_wps[mis_idx]
                            wp_type_s = "P" if (len(wp) > 3 and wp[3] == WP_PATH) else "D"
                            d = hav_dist(gps_lat, gps_lon, wp[0], wp[1])
                            tx_now("ACK,STATUS,MISSION,%d,%d,%s,%.1f,%.1fm,%s" % (
                                mis_idx, len(mission_wps),
                                PHASE_NAME[mis_phase], d, est_depth, wp_type_s))
                        elif nav_on and tgt_lat is not None:
                            d = hav_dist(gps_lat, gps_lon, tgt_lat, tgt_lon)
                            tx_now("ACK,STATUS,NAV,%.6f,%.6f,%.1f" % (tgt_lat, tgt_lon, d))
                        elif holding:
                            d = hav_dist(gps_lat, gps_lon, hold_lat, hold_lon)
                            tx_now("ACK,STATUS,HOLDING,%.6f,%.6f,%.1f" % (hold_lat, hold_lon, d))
                        else: tx_now("ACK,STATUS,IDLE")

                    elif c == "SET_NORTH" and len(p) >= 2:
                        try:
                            true_bearing = float(p[1])
                            if 0 <= true_bearing < 360:
                                raw_hdg = (heading + HDG_OFF) % 360.0
                                HDG_OFF = (raw_hdg - true_bearing) % 360.0
                                tx_now("ACK,SET_NORTH,%.1f,HDG_OFF=%.1f" % (true_bearing, HDG_OFF))
                            else: tx_now("NAK,SET_NORTH,RANGE")
                        except: tx_now("NAK,SET_NORTH,PARSE")

                    elif c == "GET_NORTH":
                        raw_hdg = (heading + HDG_OFF) % 360.0
                        tx_now("ACK,GET_NORTH,HDG_OFF=%.1f,RAW=%.1f,CORRECTED=%.1f" % (
                            HDG_OFF, raw_hdg, heading))

                    elif c == "SAFETY_OFF" and len(p) >= 2:
                        name = p[1].lower()
                        valid = ("compass", "gps", "battery", "dht", "mtemp", "pod", "hull", "motor")
                        if name not in valid: tx_now("NAK,SAFETY_OFF,UNKNOWN,%s" % name)
                        elif name in CRITICAL_OVERRIDES and (len(p) < 3 or p[2] != "CONFIRM"):
                            tx_now("NAK,SAFETY_OFF,NEED_CONFIRM,%s" % name)
                        else:
                            safety_overrides.add(name)
                            tx_now("ACK,SAFETY_OFF,%s" % name)

                    elif c == "SAFETY_ON" and len(p) >= 2:
                        safety_overrides.discard(p[1].lower())
                        tx_now("ACK,SAFETY_ON,%s" % p[1].lower())

                    elif c == "SAFETY_ALL_ON":
                        safety_overrides.clear(); tx_now("ACK,SAFETY_ALL_ON")

                    elif c == "SAFETY":
                        if safety_overrides: tx_now("ACK,SAFETY,%s" % '+'.join(safety_overrides))
                        else: tx_now("ACK,SAFETY,ALL_ACTIVE")

                    elif c == "RANGE" and len(p) >= 3:
                        try:
                            r_lat = float(p[1]); r_lon = float(p[2])
                            dist = hav_dist(gps_lat, gps_lon, r_lat, r_lon)
                            v_avail = batt_v - B_CRIT
                            if eng_v_per_m > 0 and v_avail > 0:
                                max_range = v_avail / eng_v_per_m
                                v_needed = dist * eng_v_per_m
                                margin = v_avail - v_needed
                                ok = "YES" if margin > 0 else "NO"
                                tx_now("ACK,RANGE,%s,%.0fm,%.2fV_need,%.2fV_avail,%.0fm_max" % (
                                    ok, dist, v_needed, v_avail, max_range))
                            elif eng_v_per_m <= 0: tx_now("NAK,RANGE,NO_DATA,%.0fm" % dist)
                            else: tx_now("NAK,RANGE,NO_BATT,%.0fm" % dist)
                        except: tx_now("NAK,RANGE,PARSE")

                    elif c == "SET_TRIM" and len(p) >= 2:
                        try:
                            t_val = float(p[1])
                            if -0.5 <= t_val <= 0.5:
                                TRIM = t_val
                                tx_now("ACK,SET_TRIM,%.3f,L=%.3f,R=%.3f" % (
                                    TRIM, COMP_L + TRIM, COMP_R - TRIM))
                            else: tx_now("NAK,SET_TRIM,RANGE")
                        except: tx_now("NAK,SET_TRIM,PARSE")

                    elif c == "GET_TRIM":
                        tx_now("ACK,GET_TRIM,%.3f,L=%.3f,R=%.3f,PULLEY_L=%.3f,PULLEY_R=%.3f" % (
                            TRIM, COMP_L + TRIM, COMP_R - TRIM, PULLEY_L, PULLEY_R))

                    elif c == "SET_SPEED" and len(p) >= 3:
                        try:
                            target = p[1].upper(); val = int(p[2])
                            if target == "NAV" and 0 < val <= 100:
                                NAV_SPD = val; tx_now("ACK,SET_SPEED,NAV,%d" % NAV_SPD)
                            elif target == "MANUAL" and 0 < val <= 100:
                                MANUAL_SPD = val; tx_now("ACK,SET_SPEED,MANUAL,%d" % MANUAL_SPD)
                            elif target == "WINCH" and 0 < val <= 100:
                                WINCH_SPD = val; W_DN_SPD = val; W_UP_SPD = -val
                                tx_now("ACK,SET_SPEED,WINCH,%d" % WINCH_SPD)
                            elif target == "ALL" and 0 < val <= 100:
                                NAV_SPD = val; MANUAL_SPD = val; WINCH_SPD = val
                                W_DN_SPD = val; W_UP_SPD = -val
                                tx_now("ACK,SET_SPEED,ALL,%d" % val)
                            else: tx_now("NAK,SET_SPEED,RANGE")
                        except: tx_now("NAK,SET_SPEED,PARSE")

                    elif c == "GET_SPEED":
                        tx_now("ACK,GET_SPEED,NAV=%d,MANUAL=%d,WINCH=%d" % (
                            NAV_SPD, MANUAL_SPD, WINCH_SPD))

                    else: tx_now("NAK,UNKNOWN,%s" % c)

        except Exception as e: tx("STATUS,ERR,cmd,%s" % e)
        await asyncio.sleep(0.05)

async def task_radio_watchdog():
    global radio_lost, rc_over
    while radio_last_rx == 0: await asyncio.sleep(1)
    while True:
        try:
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, radio_last_rx)
            if elapsed > RADIO_LOST_MS and not radio_lost:
                radio_lost = True
                tx("STATUS,RADIO_LOST,%ds" % (elapsed // 1000))
                if not rc_over: rc_on(); tx("STATUS,RC_AUTO,LINK_LOST")
            elif elapsed <= RADIO_LOST_MS and radio_lost:
                radio_lost = False; tx("STATUS,RADIO_RESTORED")
        except: pass
        await asyncio.sleep(2)

def radio_selftest():
    while radio.any(): radio.read()
    time.sleep_ms(200)
    if radio.any():
        junk = radio.read()
        if junk:
            bad = sum(1 for b in junk if b >= 0xF0)
            if bad > len(junk) // 2:
                print("!! RADIO RX WARNING: noise on idle line")
                return False
    print("Radio RX line: quiet (OK)")
    return True

async def main():
    print("Boat v4.6 starting")
    set_m(ml, 0); set_m(mr, 0); set_m(mw, 0)
    await asyncio.sleep(2)
    init_compass()
    now = time.ticks_ms()
    for name in hlth: hlth[name][2] = now
    if i2c is not None:
        try: print("I2C: %s" % [hex(d) for d in i2c.scan()])
        except: pass
    print("Radio UART0 TX:GP12 RX:GP13 9600bd")
    print("Nav: GPS track primary, compass fallback")
    print("Radio watchdog: %ds timeout -> RC override" % (RADIO_LOST_MS // 1000))
    radio_selftest()
    print("Ready")
    await asyncio.gather(
        task_radio_tx(), task_fast_sensors(), task_slow_sensors(),
        task_motors(), task_telemetry(), task_navigate(),
        task_mission(), task_watchdogs(), task_commands(),
        task_radio_watchdog()
    )

asyncio.run(main())
