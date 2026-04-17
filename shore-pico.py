from machine import Pin, UART
import uasyncio as asyncio
import time
import json

try:    radio = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
except: radio = None
try:    btn = Pin(15, Pin.IN, Pin.PULL_UP)
except: btn = None
try:    estop = Pin(16, Pin.IN, Pin.PULL_UP)
except: estop = None

input_mode = False
MAX_RETRY = 5
last_rx = 0; boat_up = False; boat_lost_sent = False

LOG = "survey.csv"; ERR_LOG = "errors.csv"; PROF_LOG = "profiles.csv"
WAYPOINT_FILE = "waypoints.json"
log_n = 0

# COMMAND RESPONSE STATE
cmd_pending = False
cmd_result = None
cmd_resp = ""
last_ack = None

# MISSION / HOME STATE
home_lat = None; home_lon = None; home_name = ""
missions = {}
rth_prompted = False
rth_active = False

def load_waypoints():
    global home_lat, home_lon, home_name, missions
    try:
        f = open(WAYPOINT_FILE, "r")
        cfg = json.load(f)
        f.close()
    except Exception as e:
        print("  ! Could not load %s: %s" % (WAYPOINT_FILE, e))
        return False

    if "home" in cfg:
        h = cfg["home"]
        home_lat = h.get("lat"); home_lon = h.get("lon")
        home_name = h.get("name", "Home")
        if home_lat is not None and home_lon is not None:
            print("  Home: %s (%.6f, %.6f)" % (home_name, home_lat, home_lon))
        else:
            print("  ! Home waypoint missing lat/lon")
            home_lat = None; home_lon = None

    if "missions" in cfg:
        missions = cfg["missions"]
        print("  Missions loaded:")
        for key in missions:
            m = missions[key]
            n = len(m.get("waypoints", []))
            print("    %-16s  %s (%d WPs)" % (key, m.get("name", key), n))
    else:
        print("  No missions found in file")

    return True

def list_missions():
    if not missions:
        print("  No missions loaded. Check %s" % WAYPOINT_FILE)
        return
    print("\n  Available missions:")
    keys = sorted(missions.keys())
    for i, key in enumerate(keys):
        m = missions[key]
        wps = m.get("waypoints", [])
        data_n = sum(1 for w in wps if w.get("type", "data") == "data")
        path_n = len(wps) - data_n
        print("    %d) %-16s  %s (%d WPs: %dD %dP)" % (
            i, key, m.get("name", key), len(wps), data_n, path_n))
        for j, wp in enumerate(wps):
            wt = wp.get("type", "data")
            lbl = wp.get("label", "")
            marker = "[DATA]" if wt == "data" else "[PATH]"
            if wt == "data":
                mt = wp.get("max_time", 300)
                print("       WP%d %s %.6f, %.6f  max:%ds  %s" % (
                    j, marker, wp["lat"], wp["lon"], mt, lbl))
            else:
                print("       WP%d %s %.6f, %.6f  %s" % (
                    j, marker, wp["lat"], wp["lon"], lbl))
    if home_lat is not None:
        print("    Home: %s (%.6f, %.6f)" % (home_name, home_lat, home_lon))

def mission_to_cmd(mission_key):
    if mission_key not in missions:
        print("  Unknown mission: %s" % mission_key)
        return None
    m = missions[mission_key]
    wps = m.get("waypoints", [])
    if not wps:
        print("  Mission '%s' has no waypoints" % mission_key)
        return None
    parts = ["MISSION", str(len(wps))]
    for wp in wps:
        la = wp["lat"]; lo = wp["lon"]
        wt = wp.get("type", "data")
        t = wp.get("max_time", 300) if wt == "data" else 0
        wt_int = 0 if wt == "data" else 1
        if not (-90 <= la <= 90 and -180 <= lo <= 180):
            print("  Invalid coords in mission: %.6f, %.6f" % (la, lo))
            return None
        parts.extend([str(la), str(lo), str(t), str(wt_int)])
    return ",".join(parts)

def home_goto_cmd():
    if home_lat is None or home_lon is None:
        print("  No home waypoint set. Edit %s" % WAYPOINT_FILE)
        return None
    return "GOTO,%s,%s" % (home_lat, home_lon)

def home_mission_cmd():
    if home_lat is None or home_lon is None:
        print("  No home waypoint set. Edit %s" % WAYPOINT_FILE)
        return None
    return "MISSION,1,%s,%s,0,1" % (home_lat, home_lon)
    # time=0, wp_type=1 (WP_PATH) so boat arrives and stops, no winch drop

# LOGGING
def init_log():
    global log_n
    try:
        f = open(LOG, "r")
        for _ in f: log_n += 1
        log_n -= 1; f.close()
        print("  Log: %d existing rows" % log_n)
    except:
        try:
            f = open(LOG, "w")
            f.write("time,lat,lon,hdg,spd,status,batt_v,batt_pct,")
            f.write("air_t,air_h,hull,mt_l,mt_r,")
            f.write("w_temp,w_ec_uScm,light_pct,bottom,")
            f.write("quality,dist,phase,winch\n")
            f.close(); log_n = 0; print("  Log: created")
        except Exception as e:
            print("  Log failed: %s" % e)
    try: open(ERR_LOG, "r").close()
    except:
        try: f = open(ERR_LOG, "w"); f.write("time,source,error\n"); f.close()
        except: pass
    try: open(PROF_LOG, "r").close()
    except:
        try:
            f = open(PROF_LOG, "w")
            f.write("time,wp,sample,depth_m,water_temp,water_ec_uScm,light_pct,lat,lon\n")
            f.close(); print("  Profile log: created")
        except: pass

def _ts():
    t = time.ticks_ms() // 1000
    return "%02d:%02d:%02d" % (t // 3600, t % 3600 // 60, t % 60)

def log_combined():
    global log_n
    try:
        f = open(LOG, "a")
        f.write("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            _ts(), D['lat'], D['lon'], D['hdg'], D['spd'], D['st'],
            D['bv'], D['bp'], D['at'], D['ah'], D['hl'],
            D['ml'], D['mr'], D['wt'], D['we'], D['lt'], D['btm'],
            D['qf'], D['dist'], D['phase'], D['winch']))
        f.close(); log_n += 1
    except: pass

def log_error(source, error):
    try:
        f = open(ERR_LOG, "a")
        f.write("%s,%s,%s\n" % (_ts(), source, error))
        f.close()
    except: pass

def log_profile(parts):
    if len(parts) < 8: return
    try:
        f = open(PROF_LOG, "a")
        f.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
            _ts(), parts[2], parts[3], parts[4], parts[5],
            parts[6], parts[7], D['lat'], D['lon']))
        f.close()
        evt("~", "Profile WP%s #%s: D:%sm T:%s EC:%s L:%s%%" % (
            parts[2], parts[3], parts[4], parts[5], parts[6], parts[7]))
    except: pass

def log_skip(wp_idx, phase, samples, depth, elapsed):
    """Log a skip event so downstream analysis knows this WP was partial."""
    try:
        f = open(PROF_LOG, "a")
        f.write("%s,%s,SKIPPED,%s,%s,phase=%s,,,%s,%s\n" % (
            _ts(), wp_idx, depth, elapsed, phase, D['lat'], D['lon']))
        f.close()
    except: pass

# RADIO FRAMING
rx_buf = bytearray()

def _cs(s):
    c = 0
    for ch in s: c ^= ord(ch)
    return c

def frame(msg):
    return "$%s*%d\n" % (msg, _cs(msg))

def parse(raw):
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
        fr = rx_buf[1:ei]; rx_buf = rx_buf[ei + 1:]
        try:
            s = fr.decode('utf-8', 'ignore').strip()
            if '*' in s:
                p, c = s.rsplit('*', 1)
                if _cs(p) == int(c): out.append(p)
        except: pass
    return out

# COMMS

# FIX: wait for a gap in incoming telemetry before sending command
# This detects when the boat is in its listen window.
async def _wait_for_gap(timeout_ms=6000):
    """Wait until we see a gap in incoming data (boat listen window)."""
    if radio is None: return False
    start = time.ticks_ms()
    # First, wait until we see some data (boat is transmitting)
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if radio.any():
            break
        await asyncio.sleep(0.02)
    # Now wait for the data to stop (boat entered listen window)
    last_data = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if radio.any():
            # Drain it so task_rx can process later, but note activity
            last_data = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), last_data) > 300:
            # 300ms silence = boat is likely in listen mode
            return True
        await asyncio.sleep(0.02)
    return False  # timed out

async def send_blind(cmd, repeats=3):
    if radio is None: print("  No radio"); return
    framed = frame(cmd)
    print("  > %s (blind x%d)" % (cmd, repeats))
    for i in range(repeats):
        await _wait_for_gap(4000)
        try: radio.write(framed)
        except: print("  Radio write failed"); return
        await asyncio.sleep(0.5)

async def send_cmd(cmd):
    global cmd_pending, cmd_result, cmd_resp, last_ack
    if radio is None: print("  No radio"); return False
    framed = frame(cmd)
    cmd_tag = cmd.split(",")[0]  # e.g. "ALL_STOP", "GET_NORTH", "STATUS"
    for attempt in range(MAX_RETRY):
        cmd_result = None
        cmd_resp = ""
        last_ack = None
        cmd_pending = True

        # Wait for boat's listen window before transmitting
        await _wait_for_gap(4000)

        try: radio.write(framed)
        except: print("  Radio write failed"); cmd_pending = False; return False
        print("  > %s (%d)" % (cmd, attempt + 1))

        # Small delay for HC-12 to finish transmitting our command
        await asyncio.sleep(0.15)

        # FIX: send command twice for reliability — HC-12 half-duplex
        # can lose the first one if the boat is just finishing a TX
        try: radio.write(framed)
        except: pass
        await asyncio.sleep(0.1)

        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 5000:
            if cmd_result is not None:
                cmd_pending = False
                print("  < %s" % cmd_resp)
                return cmd_result
            if last_ack is not None:
                cmd_pending = False
                print("  < %s" % last_ack[1])
                result = last_ack[0]
                last_ack = None
                return result
            await asyncio.sleep(0.02)
        print("  . timeout")
        await asyncio.sleep(0.5)
    cmd_pending = False
    print("  ! No ACK -- sending blind")
    await send_blind(cmd, 5)
    return True

# DISPLAY
D = {
    "lat": "---", "lon": "---", "hdg": "---", "spd": "---", "st": "---",
    "bv": "---", "bp": "---", "at": "---", "ah": "---", "hl": "---",
    "ml": "---", "mr": "---", "wt": "---", "we": "---", "lt": "---", "btm": "",
    "qf": "------", "dist": "", "phase": "", "winch": "",
    "d1_ok": False, "d2_ok": False,
    "vpm": 0.0, "eng_dist": 0.0, "vpkm": 0.0, "eng_ok": False
}
QF_NAMES = ("GPS", "CMP", "BAT", "AIR", "MOT", "POD")

def show():
    conn = "LINK" if boat_up else "----"
    qraw = D['qf'].replace("Q", "")
    warns = ""
    for i in range(min(len(qraw), len(QF_NAMES))):
        if qraw[i] == "0": warns += " %s!" % QF_NAMES[i]
    print("---------------------------------------------")
    print("  POS %s, %s" % (D['lat'], D['lon']))
    print("  HDG %s SPD %s [%s] %s" % (D['hdg'], D['spd'], D['st'], conn))
    print("  BAT %sV %s%%  HULL %s" % (D['bv'], D['bp'], D['hl']))
    print("  AIR %sC %s%%RH  MOT L:%sC R:%sC" % (D['at'], D['ah'], D['ml'], D['mr']))
    sea = "  SEA T:%sC EC:%suS/cm L:%s%%" % (D['wt'], D['we'], D['lt'])
    if D['btm']: sea += " ** BOTTOM **"
    print(sea)
    ex = ""
    if D['dist']:  ex += " DIST:%sm" % D['dist']
    if D['phase']: ex += " %s" % D['phase']
    if D['winch']: ex += " W:%s" % D['winch']
    if ex: print(ex)
    if warns: print("  WARN%s" % warns)
    if D['eng_ok']:
        print("  ENERGY %.4fV/m (%.2fV/km) tracked:%.0fm" % (
            D['vpm'], D['vpkm'], D['eng_dist']))
        if home_lat is not None and D['vpm'] > 0:
            try:
                blat = float(D['lat']); blon = float(D['lon'])
                bv = float(D['bv'])
                import math
                r1 = blat * 3.14159265 / 180; r2 = home_lat * 3.14159265 / 180
                dlat = (home_lat - blat) * 3.14159265 / 180
                dlon = (home_lon - blon) * 3.14159265 / 180
                a = math.sin(dlat/2)**2 + math.cos(r1)*math.cos(r2)*math.sin(dlon/2)**2
                home_dist = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                v_need = home_dist * D['vpm']
                v_avail = bv - 13.6
                if v_avail > 0:
                    max_range = v_avail / D['vpm']
                    margin_pct = ((v_avail - v_need) / v_avail) * 100
                    status = "OK" if v_need < v_avail else "LOW"
                    print("  HOME %.0fm need:%.2fV avail:%.2fV max:%.0fm [%s %.0f%%]" % (
                        home_dist, v_need, v_avail, max_range, status, margin_pct))
                else:
                    print("  HOME %.0fm -- NO VOLTAGE MARGIN" % home_dist)
            except: pass
    if rth_active: print("  ** RETURNING HOME **")
    print("  LOG %d rows" % log_n)
    print("---------------------------------------------")

def evt(icon, msg):
    t = time.ticks_ms() // 1000
    print("  %s [%02d:%02d] %s" % (icon, t // 60 % 60, t % 60, msg))

def alert(msg):
    print("\n  >>> %s <<<\n" % msg)

# RETURN-TO-HOME LOGIC
async def prompt_return_home(voltage, pct):
    global rth_prompted, rth_active, input_mode
    if rth_prompted or rth_active: return
    if home_lat is None:
        alert("BATT LOW %sV %s%% -- no home WP set!" % (voltage, pct))
        rth_prompted = True
        return

    alert("BATT LOW %sV %s%% -- RETURN HOME?" % (voltage, pct))
    print("  Home: %s (%.6f, %.6f)" % (home_name, home_lat, home_lon))

    if D['eng_ok'] and D['vpm'] > 0:
        try:
            blat = float(D['lat']); blon = float(D['lon'])
            bv = float(D['bv'])
            import math
            r1 = blat * 3.14159265 / 180; r2 = home_lat * 3.14159265 / 180
            dlat = (home_lat - blat) * 3.14159265 / 180
            dlon = (home_lon - blon) * 3.14159265 / 180
            a = math.sin(dlat/2)**2 + math.cos(r1)*math.cos(r2)*math.sin(dlon/2)**2
            home_dist = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            v_need = home_dist * D['vpm']
            v_avail = bv - 13.6
            if v_avail > 0 and v_need < v_avail:
                print("  Estimate: %.0fm, need %.2fV of %.2fV available -- REACHABLE" % (
                    home_dist, v_need, v_avail))
            elif v_avail > 0:
                print("  Estimate: %.0fm, need %.2fV but only %.2fV available -- MARGINAL" % (
                    home_dist, v_need, v_avail))
            else:
                print("  Estimate: NO voltage margin above critical!")
        except:
            pass

    input_mode = True
    try:
        resp = input("  Return home now? (y/n): ").strip().lower()
    except:
        resp = "n"
    input_mode = False

    if resp == "y":
        rth_active = True
        rth_prompted = True
        evt("H", "RETURNING TO HOME: %s" % home_name)
        cmd = home_goto_cmd()
        if cmd:
            await send_cmd(cmd)
        else:
            alert("FAILED TO BUILD HOME COMMAND")
            rth_active = False
    else:
        rth_prompted = True
        evt("!", "Operator declined return-to-home")

# STATUS HANDLER
def _handle_status(parts):
    global rth_active, rth_prompted
    n = len(parts)
    if n < 2: return
    k = parts[1]
    if   k == "ARRIVED":
        if rth_active:
            alert("ARRIVED HOME -- %s" % home_name)
            rth_active = False
        else:
            alert("ARRIVED AT TARGET")
    elif k == "TRANSIT" and n>=5: evt(">", "Transit WP%s: %s,%s" % (parts[2], parts[3], parts[4]))
    elif k == "PATH_PASS" and n>=5: evt("-", "Path WP%s (pass-through): %s,%s" % (parts[2], parts[3], parts[4]))
    elif k == "WINCH_DOWN" and n>=3: evt("v", "Winch down WP%s" % parts[2])
    elif k == "WINCH_UP" and n>=3:
        msg = "Winch up WP%s" % parts[2]
        if n >= 4: msg += " (dn:%s)" % parts[3]
        evt("^", msg)
    elif k == "SAMPLING" and n>=4:
        evt("~", "Sample WP%s %s" % (parts[2], ','.join(parts[3:])))
    elif k == "PROFILE" and n>=7:
        evt("~", "WP%s D:%sm T:%sC EC:%suS L:%s%%" % (parts[2], parts[3], parts[4], parts[5], parts[6]))
    elif k == "PDATA" and n>=8:    log_profile(parts)
    elif k == "SAMPLE_DATA" and n>=8:
        evt("~", "WP%s #%s D:%sm T:%sC EC:%suS L:%s%% %s" % (
            parts[2], parts[3], parts[4], parts[5], parts[6], parts[7],
            parts[8] if n >= 9 else ""))
        log_profile(parts)
    elif k == "CONTACT_LOST" and n>=4:
        alert("CONTACT LOST WP%s -- %s in %s" % (parts[2], parts[3], parts[4] if n>=5 else ""))
    elif k == "SAMPLE_TIMEOUT" and n>=4:
        alert("SAMPLE TIMEOUT WP%s -- %s in %s" % (parts[2], parts[3], parts[4] if n>=5 else ""))
    elif k == "SKIP_DROP" and n>=6:
        alert("SKIP DROP WP%s phase:%s %s depth:%s time:%s" % (
            parts[2], parts[3], parts[4], parts[5], parts[6] if n>=7 else ""))
        log_skip(parts[2], parts[3], parts[4], parts[5], parts[6] if n>=7 else "?")
    elif k == "BOTTOM_CONFIRMED" and n>=5:
        alert("BOTTOM CONFIRMED WP%s depth:%sm time:%s" % (parts[2], parts[3], parts[4]))
    elif k == "BOTTOM_PROBE" and n>=5:
        evt("v", "Probe %s/3 WP%s D:%sm" % (parts[3], parts[2], parts[4]))
    elif k == "MISSION_COMPLETE" and n>=3:
        if rth_active:
            alert("HOME REACHED -- %s WPs done" % parts[2])
            rth_active = False
        else:
            alert("MISSION COMPLETE -- %s WPs done" % parts[2])
    elif k == "MISSION_ABORTED":   alert("MISSION ABORTED")
    elif k == "MISSION_REPLACED":  evt("-", "Mission replaced")
    elif k == "MISSION_RC_OVERRIDE": alert("MISSION STOPPED -- RC")
    elif k == "MISSION_BATT_CRITICAL": alert("MISSION STOPPED -- BATTERY")
    elif k == "MISSION_COMPASS_FAULT": alert("MISSION STOPPED -- COMPASS")
    elif k == "MISSION_GPS_FAULT": alert("MISSION STOPPED -- GPS")
    elif k == "RC_OVERRIDE_ON":    alert("RC OVERRIDE ON")
    elif k == "RC_OVERRIDE_OFF":   alert("RC RELEASED")
    elif k == "RC_OVERRIDE":       evt("R", "RC active")
    elif k == "BATT_CRITICAL" and n>=4:
        alert("BATT CRITICAL %sV %s%%" % (parts[2], parts[3]))
    elif k == "BATT_LOW" and n>=4:
        evt("!", "BATT LOW %sV %s%%" % (parts[2], parts[3]))
    elif k == "SAFE_STOP" and n>=3:  alert("SAFE STOP: %s" % parts[2])
    elif k == "HULL_LEAK" and n>=3:  alert("HULL LEAK: %s" % parts[2])
    elif k == "HULL_DRY" and n>=3:   evt("+", "%s hull dry" % parts[2])
    elif k == "MOTOR_HOT" and n>=4:  evt("!", "Motor hot %s %sC" % (parts[2], parts[3]))
    elif k == "MOTOR_COOL" and n>=3: evt("+", "Motors cool %sC" % parts[2])
    elif k == "MOTOR_OVERHEAT" and n>=4:
        alert("MOTOR OVERHEAT %s %sC" % (parts[2], parts[3]))
    elif k == "SENSOR_FAULT" and n>=3: evt("!", "FAULT: %s" % parts[2])
    elif k == "SENSOR_OK" and n>=3:    evt("+", "OK: %s" % parts[2])
    elif k == "HEALTH" and n>=4 and parts[2] == "FAULT":
        evt("!", "Faults: %s" % parts[3])
    elif k == "MISSION" and n>=7:
        evt("i", "WP%s/%s %s D:%sm W:%s" % (parts[2], parts[3], parts[4], parts[5], parts[6]))
    elif k == "NAV" and n>=5:
        evt("i", "Nav %s,%s D:%sm" % (parts[2], parts[3], parts[4]))
    elif k == "HOLDING" and n>=5:
        evt("H", "Holding %s,%s drift:%sm" % (parts[2], parts[3], parts[4]))
    elif k == "HOLD_REACQUIRE" and n>=3:
        evt("H", "Re-acquiring position, drift:%sm" % parts[2])
    elif k == "SAFETY_OVERRIDES" and n>=3:
        evt("!", "SAFETY OVERRIDES ACTIVE: %s" % parts[2])
    elif k == "SAFETY_ALL_ACTIVE":
        evt("+", "All safety systems active")
    elif k == "NORTH" and n >= 4:
        evt("N", "Compass: %s %s %s" % (parts[2], parts[3], parts[4]))
    elif k == "TRIM" and n >= 6:
        evt("T", "Trim:%s L=%s R=%s PL=%s PR=%s" % (
            parts[2], parts[3], parts[4], parts[5], parts[6]))
    elif k == "WARN_OVERRIDE" and n>=3:
        warn_msg = "OVERRIDE: %s fault ignored" % parts[2]
        if n >= 4: warn_msg += " (%s)" % parts[3]
        evt("!", warn_msg)
    elif k == "IDLE":              evt("i", "Idle")
    elif k == "RADIO_LOST" and n>=3:
        alert("BOAT LOST RADIO LINK (%s)" % parts[2])
    elif k == "RADIO_RESTORED":
        alert("BOAT RADIO LINK RESTORED")
    elif k == "RC_AUTO" and n>=3:
        if parts[2] == "LINK_LOST":
            alert("BOAT AUTO RC-OVERRIDE — LINK LOST")
        elif parts[2] == "LINK_RESTORED":
            alert("BOAT AUTO RC-RELEASED — LINK RESTORED")
    elif k == "ERR" and n>=4:
        evt("!", "ERROR %s: %s" % (parts[2], parts[3]))
        log_error(parts[2], parts[3])
    else:
        print("  %s" % ','.join(parts))

# RECEIVE TASK
batt_low_pending = False

async def task_rx():
    global last_rx, boat_up, boat_lost_sent, batt_low_pending
    global cmd_result, cmd_resp, last_ack
    while True:
        try:
            if radio is None: await asyncio.sleep(0.1); continue
            if radio.any():
                try: raw = radio.read()
                except: await asyncio.sleep(0.02); continue
                if not raw: await asyncio.sleep(0.02); continue
                for msg in parse(raw):
                    last_rx = time.ticks_ms()
                    if not boat_up:
                        boat_up = True; boat_lost_sent = False
                        alert("BOAT CONNECTED")

                    if msg[:4] == "ACK," or msg[:4] == "NAK,":
                        is_ack = msg[:4] == "ACK,"
                        if cmd_pending:
                            cmd_result = is_ack; cmd_resp = msg
                        else:
                            last_ack = (is_ack, msg)
                        continue

                    if msg[:7] == "STATUS,":
                        parts = msg.split(",")
                        _handle_status(parts)
                        if len(parts) >= 4 and parts[1] == "BATT_LOW":
                            batt_low_pending = True
                        if len(parts) >= 4 and parts[1] == "BATT_CRITICAL":
                            if not rth_active and home_lat is not None:
                                alert("AUTO RETURN HOME -- BATTERY CRITICAL")
                                rth_active = True
                        continue
                    p = msg.split(",")
                    if p[0] == "D1" and len(p) >= 10:
                        D['lat']=p[1]; D['lon']=p[2]; D['hdg']=p[3]; D['spd']=p[4]
                        D['st']=p[5]; D['bv']=p[6]; D['bp']=p[7]
                        D['hl']=p[8]; D['qf']=p[9]
                        D['dist']  = p[10] if len(p) >= 11 else ""
                        D['phase'] = p[11] if len(p) >= 12 else ""
                        D['winch'] = p[12] if len(p) >= 13 else ""
                        D['d1_ok'] = True
                        if D['d2_ok']:
                            show(); log_combined()
                            D['d1_ok'] = False; D['d2_ok'] = False
                    elif p[0] == "D2" and len(p) >= 8:
                        D['at']=p[1]; D['ah']=p[2]; D['ml']=p[3]; D['mr']=p[4]
                        D['wt']=p[5]; D['we']=p[6]; D['lt']=p[7]
                        D['btm'] = p[8] if len(p) >= 9 else ""
                        D['d2_ok'] = True
                        if D['d1_ok']:
                            show(); log_combined()
                            D['d1_ok'] = False; D['d2_ok'] = False
                    elif p[0] == "D3" and len(p) >= 4:
                        try:
                            D['vpm'] = float(p[1])
                            D['eng_dist'] = float(p[2])
                            D['vpkm'] = float(p[3])
                            D['eng_ok'] = True
                        except: pass
        except Exception as e:
            print("  RX error: %s" % e)
        await asyncio.sleep(0.02)

async def task_conn():
    global boat_up, boat_lost_sent
    while True:
        if last_rx > 0 and time.ticks_diff(time.ticks_ms(), last_rx) > 10000 and boat_up:
            boat_up = False
            if not boat_lost_sent:
                boat_lost_sent = True; alert("CONNECTION LOST")
        await asyncio.sleep(2)

# BATTERY WATCH TASK
async def task_batt_watch():
    global batt_low_pending
    while True:
        if batt_low_pending and not input_mode:
            batt_low_pending = False
            v = D.get('bv', '?'); p = D.get('bp', '?')
            await prompt_return_home(v, p)
        await asyncio.sleep(1)

async def task_auto_rth():
    global rth_active
    sent = False
    while True:
        if rth_active and not sent and not input_mode and not cmd_pending:
            cmd = home_goto_cmd()
            if cmd:
                result = await send_cmd(cmd)
                if result:
                    evt("H", "AUTO RTH: GOTO home sent")
                    sent = True
                else:
                    alert("AUTO RTH: failed to send GOTO")
                    rth_active = False
            else:
                rth_active = False
        if not rth_active:
            sent = False
        await asyncio.sleep(1)

async def task_estop():
    last_press = 0
    while True:
        try:
            if estop is not None and estop.value() == 0:
                now = time.ticks_ms()
                if time.ticks_diff(now, last_press) > 2000:
                    last_press = now
                    alert("EMERGENCY STOP")
                    rth_active = False
                    if not input_mode:
                        await send_cmd("ALL_STOP")
                    else:
                        # Input mode — blast directly, can't use send_cmd
                        framed = frame("ALL_STOP")
                        for _ in range(5):
                            try: radio.write(framed)
                            except: pass
                            await asyncio.sleep(0.15)
                        print("  E-STOP sent 5x (blind, input active)")
        except: pass
        await asyncio.sleep(0.05)

async def task_heartbeat():
    """Send periodic ping so boat knows shore link is alive."""
    while True:
        if radio is not None and boat_up and not cmd_pending and not input_mode:
            try:
                radio.write(frame("PING"))
            except: pass
        await asyncio.sleep(10)

# INPUT TASK
def get_waypoints():
    wps = []
    print("\n  Waypoints (empty lat to finish)")
    while True:
        try: la = input("  WP%d Lat: " % len(wps)).strip()
        except: break
        if not la: break
        try:
            lo = input("  WP%d Lon: " % len(wps)).strip()
            ts = input("  WP%d Time(s)[60]: " % len(wps)).strip()
            la = float(la); lo = float(lo)
            t = int(ts) if ts else 60
            if -90 <= la <= 90 and -180 <= lo <= 180 and 1 <= t <= 3600:
                wps.append((la, lo, t)); print("  + %s,%s %ds" % (la, lo, t))
            else: print("  Invalid")
        except: print("  Invalid")
    return wps

async def task_input():
    global input_mode, rth_active, rth_prompted, home_lat, home_lon, home_name
    while True:
        try:
            if btn is None: await asyncio.sleep(1); continue
            if btn.value() == 0:
                input_mode = True
                print("\n  === COMMANDS (telemetry paused while typing) ===")
                print("  FORWARD BACKWARD LEFT RIGHT STOP ALL_STOP")
                print("  WINCH_DOWN WINCH_UP WINCH_STOP")
                print("  GOTO CANCEL_NAV")
                print("  MISSION MISSION_START MISSION_ABORT")
                print("  LOAD <n>            -- load mission from file")
                print("  MISSIONS            -- list available missions")
                print("  HOME                -- return to home waypoint")
                print("  SET_HOME <lat> <lon> -- override home position")
                print("  RANGE               -- can boat reach home?")
                print("  RANGE <lat> <lon>   -- can boat reach target?")
                print("  TRIM <value>        -- set motor trim (-0.5 to +0.5)")
                print("  TRIM                -- show current trim and pulley info")
                print("  SPEED               -- show current speeds")
                print("  SPEED <type> <0-100> -- set speed (nav/manual/winch/all)")
                print("  RC_OVERRIDE RC_RELEASE STATUS")
                print("  SAFETY SAFETY_OFF <n> SAFETY_ON <n> SAFETY_ALL_ON")
                print("  SET_NORTH <bearing>  GET_NORTH")
                print("  RELOAD              -- reload waypoints.json")
                print("    sensor names: compass gps battery dht mtemp pod hull motor\n")
                try: cmd = input("  > ").strip()
                except: input_mode = False; await asyncio.sleep(0.3); continue

                cmd_upper = cmd.upper()

                if cmd_upper == "GOTO":
                    try:
                        la = float(input("  Lat: ").strip())
                        lo = float(input("  Lon: ").strip())
                        if -90 <= la <= 90 and -180 <= lo <= 180:
                            await send_cmd("GOTO,%s,%s" % (la, lo))
                        else: print("  Invalid range")
                    except: print("  Invalid")

                elif cmd_upper == "MISSION":
                    wps = get_waypoints()
                    if wps:
                        print("\n  %d waypoints:" % len(wps))
                        for i, w in enumerate(wps):
                            print("    %d: %s,%s %ds" % (i, w[0], w[1], w[2]))
                        try: ok = input("  Upload? (y/n): ").strip().lower()
                        except: ok = "n"
                        if ok == "y":
                            parts = ["MISSION", str(len(wps))]
                            for w in wps:
                                parts.extend([str(w[0]), str(w[1]), str(w[2])])
                            await send_cmd(",".join(parts))

                elif cmd_upper.startswith("LOAD"):
                    parts = cmd.split(None, 1)
                    if len(parts) < 2:
                        print("  Usage: LOAD <mission_name>")
                        list_missions()
                    else:
                        key = parts[1].strip().lower()
                        keys = sorted(missions.keys())
                        try:
                            idx = int(key)
                            if 0 <= idx < len(keys): key = keys[idx]
                        except: pass
                        mission_cmd = mission_to_cmd(key)
                        if mission_cmd:
                            m = missions[key]
                            wps = m.get("waypoints", [])
                            data_n = sum(1 for w in wps if w.get("type", "data") == "data")
                            path_n = len(wps) - data_n
                            print("\n  Loading: %s (%d data, %d path)" % (
                                m.get("name", key), data_n, path_n))
                            for i, wp in enumerate(wps):
                                wt = wp.get("type", "data")
                                lbl = wp.get("label", "")
                                marker = "[DATA]" if wt == "data" else "[PATH]"
                                if wt == "data":
                                    print("    WP%d %s %.6f, %.6f  %ds  %s" % (
                                        i, marker, wp["lat"], wp["lon"], wp.get("time", 60), lbl))
                                else:
                                    print("    WP%d %s %.6f, %.6f  %s" % (
                                        i, marker, wp["lat"], wp["lon"], lbl))
                            try: ok = input("  Upload? (y/n): ").strip().lower()
                            except: ok = "n"
                            if ok == "y":
                                result = await send_cmd(mission_cmd)
                                if result:
                                    try:
                                        start = input("  Start now? (y/n): ").strip().lower()
                                    except: start = "n"
                                    if start == "y":
                                        await send_cmd("MISSION_START")

                elif cmd_upper == "MISSIONS":
                    list_missions()

                elif cmd_upper == "HOME":
                    if home_lat is None:
                        print("  No home waypoint set. Edit %s" % WAYPOINT_FILE)
                    else:
                        print("  Return to %s (%.6f, %.6f)?" % (home_name, home_lat, home_lon))
                        try: ok = input("  Confirm (y/n): ").strip().lower()
                        except: ok = "n"
                        if ok == "y":
                            # Use GOTO instead of mission for simple return
                            cmd = home_goto_cmd()
                            if cmd:
                                result = await send_cmd(cmd)
                                if result:
                                    rth_active = True
                                    evt("H", "RETURNING HOME: %s" % home_name)
                                else:
                                    print("  Failed to send home command")

                elif cmd_upper.startswith("SET_HOME"):
                    parts = cmd_upper.split()
                    if len(parts) == 3:
                        try:
                            home_lat = float(parts[1])
                            home_lon = float(parts[2])
                            home_name = "Manual"
                            rth_prompted = False
                            print("  Home set: %.6f, %.6f" % (home_lat, home_lon))
                        except: print("  Invalid coords")
                    else:
                        print("  Usage: SET_HOME <lat> <lon>")
                        if home_lat is not None:
                            print("  Current home: %s (%.6f, %.6f)" % (home_name, home_lat, home_lon))

                elif cmd_upper == "RELOAD":
                    rth_prompted = False
                    load_waypoints()

                elif cmd_upper.startswith("RANGE"):
                    parts = cmd_upper.split()
                    if len(parts) == 3:
                        await send_cmd("RANGE,%s,%s" % (parts[1], parts[2]))
                    elif len(parts) == 1:
                        if home_lat is not None:
                            await send_cmd("RANGE,%s,%s" % (home_lat, home_lon))
                        else:
                            print("  No home set. Use RANGE <lat> <lon>")
                    else:
                        print("  Usage: RANGE or RANGE <lat> <lon>")

                elif cmd_upper.startswith("TRIM"):
                    parts = cmd_upper.split()
                    if len(parts) == 2:
                        try:
                            t_val = float(parts[1])
                            if -0.5 <= t_val <= 0.5:
                                await send_cmd("SET_TRIM,%s" % parts[1])
                            else:
                                print("  Trim range: -0.5 to +0.5")
                        except:
                            print("  Usage: TRIM <value>  (e.g. TRIM 0.05)")
                    else:
                        await send_cmd("GET_TRIM")

                elif cmd_upper.startswith("SPEED"):
                    parts = cmd_upper.split()
                    if len(parts) == 3:
                        target = parts[1]
                        try:
                            val = int(parts[2])
                            if 0 < val <= 100 and target in ("NAV", "MANUAL", "WINCH", "ALL"):
                                await send_cmd("SET_SPEED,%s,%d" % (target, val))
                            else:
                                print("  Usage: SPEED <nav/manual/winch/all> <1-100>")
                        except:
                            print("  Usage: SPEED <nav/manual/winch/all> <1-100>")
                    elif len(parts) == 1:
                        await send_cmd("GET_SPEED")
                    else:
                        print("  Usage: SPEED or SPEED <nav/manual/winch/all> <1-100>")

                elif cmd_upper in ("MISSION_START", "MISSION_ABORT", "CANCEL_NAV",
                             "FORWARD", "BACKWARD", "LEFT", "RIGHT",
                             "STOP", "ALL_STOP", "STATUS",
                             "WINCH_DOWN", "WINCH_UP", "WINCH_STOP",
                             "RC_OVERRIDE", "RC_RELEASE",
                             "SAFETY", "SAFETY_ALL_ON", "GET_NORTH"):
                    if cmd_upper in ("MISSION_ABORT", "CANCEL_NAV", "ALL_STOP"):
                        rth_active = False
                    await send_cmd(cmd_upper)

                elif cmd_upper.startswith("SAFETY_OFF"):
                    parts = cmd_upper.split()
                    if len(parts) < 2:
                        print("  Usage: SAFETY_OFF <sensor>")
                    else:
                        name = parts[1].lower()
                        critical = ("battery", "hull")
                        if name in critical:
                            print("  *** WARNING: %s is a critical safety system ***" % name.upper())
                            try: confirm = input("  Type YES to confirm override: ").strip()
                            except: confirm = ""
                            if confirm == "YES":
                                await send_cmd("SAFETY_OFF,%s,CONFIRM" % name)
                            else:
                                print("  Override cancelled")
                        else:
                            await send_cmd("SAFETY_OFF,%s" % name)

                elif cmd_upper.startswith("SAFETY_ON"):
                    parts = cmd_upper.split()
                    if len(parts) < 2:
                        print("  Usage: SAFETY_ON <sensor>")
                    else:
                        await send_cmd("SAFETY_ON,%s" % parts[1].lower())

                elif cmd_upper.startswith("SET_NORTH"):
                    parts = cmd_upper.split()
                    if len(parts) < 2:
                        print("  Point boat at a known bearing, then:")
                        print("  SET_NORTH 0     -- boat is pointing true north")
                        print("  SET_NORTH 90    -- boat is pointing due east")
                        print("  SET_NORTH 180   -- boat is pointing due south")
                    else:
                        await send_cmd("SET_NORTH,%s" % parts[1])

                elif cmd_upper == "GET_NORTH":
                    await send_cmd("GET_NORTH")

                elif cmd:
                    print("  Unknown: %s" % cmd)

                print(""); input_mode = False; await asyncio.sleep(0.3)
        except Exception as e:
            print("  Input error: %s" % e); input_mode = False
        await asyncio.sleep(0.05)

# MAIN
async def main():
    print("\n=====================")
    print("  SHORE STATION v4.4")
    print("  Radio: %s" % ("OK" if radio else "FAIL"))
    print("  Button: %s" % ("OK" if btn else "FAIL"))
    print("  E-Stop: %s" % ("OK (GP16)" if estop else "NONE"))
    print("=====================\n")
    init_log()
    load_waypoints()
    print("  Waiting for boat...\n")
    tasks = [task_rx(), task_conn(), task_batt_watch(), task_auto_rth(), task_estop(), task_heartbeat()]
    if btn is not None: tasks.append(task_input())
    await asyncio.gather(*tasks)

asyncio.run(main())
