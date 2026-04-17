from machine import Pin, ADC, UART
import time
import math

TH_NOM = 10000
TH_B = 3950
TH_R = 10000
INV_T_NOM = 1.0 / 298.15
INV_B = 1.0 / TH_B

EC_CELL_K = 1000.0
EC_SERIES = 4700

led = Pin(25, Pin.OUT, value=1)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

temp_adc = None
ec_adc = None; ec_drive = None
light_adc = None
bottom_btn = None

try: temp_adc = ADC(26)
except: pass
try:
    ec_adc = ADC(27)
    ec_drive = Pin(17, Pin.OUT, value=0)  # HIGH when reading, other electrode to GND
except: pass
try: light_adc = ADC(28)
except: pass
try: bottom_btn = Pin(21, Pin.IN, Pin.PULL_UP)
except: pass

water_temp = -999.0
salinity_ec = 0.0
light_raw = 0
bottom_contact = False

def read_temp():
    global water_temp
    if temp_adc is None:
        water_temp = -999.0; return
    try:
        total = 0
        for _ in range(8): total += temp_adc.read_u16()
        raw = total >> 3
        if raw < 100 or raw > 65400:
            water_temp = -999.0; return
        resistance = TH_R * raw / (65535.0 - raw)
        if resistance <= 0:
            water_temp = -999.0; return
        steinhart = math.log(resistance / TH_NOM) * INV_B + INV_T_NOM
        if steinhart <= 0:
            water_temp = -999.0; return
        water_temp = 1.0 / steinhart - 273.15
    except:
        water_temp = -999.0

def read_ec():
    global salinity_ec
    if ec_adc is None or ec_drive is None:
        salinity_ec = 0.0; return
    try:
        # Drive HIGH, read, then turn off to reduce plating
        ec_drive.value(1)
        time.sleep_ms(15)
        total = 0
        for _ in range(16): total += ec_adc.read_u16()
        ec_drive.value(0)
        raw = total >> 4
        voltage = raw / 65535.0 * 3.3
        if voltage > 3.2:
            # Open circuit (air) — no water
            salinity_ec = 0.0
        elif voltage < 0.05:
            salinity_ec = 99999.0
        else:
            r_water = EC_SERIES * voltage / (3.3 - voltage)
            salinity_ec = EC_CELL_K * 10000.0 / r_water if r_water > 0.1 else 99999.0
    except:
        salinity_ec = 0.0

def read_light():
    global light_raw
    if light_adc is None:
        light_raw = 0; return
    try:
        total = 0
        for _ in range(8): total += light_adc.read_u16()
        light_raw = total >> 3
    except:
        light_raw = 0

def read_bottom():
    global bottom_contact
    if bottom_btn is None:
        bottom_contact = False; return
    bottom_contact = bottom_btn.value() == 0

def _xor_cs(s):
    c = 0
    for ch in s: c ^= ord(ch)
    return c

def send_data():
    btm = 1 if bottom_contact else 0
    lt = light_raw * 100 // 65535 if light_raw > 0 else 0
    msg = "POD,%.1f,%.1f,%d,%d" % (water_temp, salinity_ec, lt, btm)
    uart.write("$%s*%d\n" % (msg, _xor_cs(msg)))

_rx_buf = bytearray(64)
_rx_pos = 0

def check_poll():
    global _rx_pos
    found = False
    while uart.any():
        b = uart.read(1)
        if not b: break
        ch = b[0]
        if ch == 0x0a:
            if _rx_pos > 4:
                try:
                    line = _rx_buf[:_rx_pos].decode('utf-8', 'ignore').strip()
                    if line and line[0] == '$' and '*' in line:
                        p, c = line[1:].rsplit('*', 1)
                        if p == "POLL" and _xor_cs(p) == int(c):
                            found = True
                except: pass
            _rx_pos = 0
        elif ch == 0x24:
            _rx_pos = 0
            _rx_buf[0] = ch
            _rx_pos = 1
        elif _rx_pos < 64:
            _rx_buf[_rx_pos] = ch
            _rx_pos += 1
    return found

def main():
    print("Sensor pod v4.3")
    print("  TX:GP0 RX:GP1 9600bd")
    print("  Temp:GP26 EC_drive:GP17 EC_adc:GP27 Light:GP28 Bottom:GP21")
    print("  EC: simple drive, 4.7k series, other electrode to GND")
    print("  EC_SERIES=%dR K=%.0f/m" % (EC_SERIES, EC_CELL_K))
    print("Ready")
    cycle = 0
    last_poll = time.ticks_ms()
    while True:
        read_temp()
        read_ec()
        read_light()
        read_bottom()
        if check_poll():
            send_data()
            last_poll = time.ticks_ms()
            led.toggle()
        elif time.ticks_diff(time.ticks_ms(), last_poll) > 30000:
            if cycle % 5 == 0:
                led.toggle()
        if cycle >= 24:
            cycle = 0
            lt = light_raw * 100 // 65535 if light_raw > 0 else 0
            btm = " BTM" if bottom_contact else ""
            print("  T:%.1fC EC:%.0fuS L:%d%%%s" %
                  (water_temp, salinity_ec, lt, btm))
        else:
            cycle += 1
        time.sleep_ms(200)

main()
