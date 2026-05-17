"""
stickersim_gpio_plc.py  –  Estación de aplicación de sticker
Versión con PLC real (Snap7) + MQTT real + pulsadores GPIO en Raspberry.

GPIO -> escribe BOOLs en DB del PLC:
- Momentáneos:
    * unabort
    * unsuspend
    * starting
    * starting_estacion
- Toggle:
    * holding
    * completing

MQTT:
- Al iniciar -> zzz=1
- starting / starting_estacion -> zzz=1
- unabort / unsuspend -> zzz=1
- holding = True -> feed=1
- completing = True -> done=1
- holding = False o completing = False -> zzz=1
- flanco positivo de aborted o suspended desde PLC -> stopped=1
- al terminar la animación -> zzz=1 con speed=20, good=..., reject=...

Controles:
  ESPACIO  → pulso de trigger (simula flanco 0→1 del PLC)
  A        → modo automático (trigger cada ~2.5 s)
  R        → resetear contadores y tiempos
  ESC / Q  → salir
"""

import pygame
import json
import math
import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

# SNAP7
import snap7
from snap7.util import get_bool, set_bool

# ─── Config ───────────────────────────────────────────────────────────────────
SIM_W   = 350
PANEL_W = 300
WIDTH   = SIM_W + PANEL_W
HEIGHT  = 550
PP      = 10

BROKER    = "192.168.1.2"
PORT      = 1883
TOPIC_PUB = "labeling/status"
TOPIC_FIRST = "scora/first"
TOPIC_RUN = "scora/run"
TOPIC_START = "scora/start"
TOPIC_STOP = "scora/stop"
TOPIC_HOME = "scora/home"

PLC_IP   = "192.168.1.33"
PLC_RACK = 0
PLC_SLOT = 1

# Trigger de ciclo leído del PLC
DB_NUM   = 1
BYTE_IDX = 3
BIT_IDX  = 1

# DB donde se escriben los comandos desde GPIO
PLC_DB_WRITE = 1

PLC_SIGNALS = {
    "unabort":           (0, 0),
    "unsuspend":         (1, 7),
    "starting":          (1, 2),
    "starting_estacion": (1, 6),
    "holding":           (0, 3),
    "completing":        (0, 5),
}

# Ajusta estos bits si aborted y suspended están en otra dirección real
PLC_STATUS_READ = {
    "aborted":   (1, 2, 3),  # (db_num, byte_idx, bit_idx)
    "suspended": (1, 2, 4),
    "starting_plc": (1, 2, 5),
}

POLL_S           = 0.05
DEBOUNCE_S       = 0.35
CYCLE_DURATION_S = 2.0

PAYLOAD_OK  = {"run":1,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":0,"speed":60,"good":0,"reject":0}
PAYLOAD_ERR = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":1,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_OFF = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_ZZZ             = {"run":0,"fault":0,"stopped":0,"zzz":1,"feed":0,"done":0,"speed":60,"good":0,"reject":0}
PAYLOAD_HOLDING_TRUE    = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":1,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_COMPLETING_TRUE = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":1,"speed":0.4,"good":0,"reject":0}
PAYLOAD_STOPPED         = {"run":0,"fault":0,"stopped":1,"zzz":0,"feed":0,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_END_ANIMATION   = {"run":0,"fault":0,"stopped":0,"zzz":1,"feed":0,"done":0,"speed":60,"good":0,"reject":0}


# ─── GPIO (BCM) ───────────────────────────────────────────────────────────────
BUTTON_PINS = {
    "unabort":           17,
    "unsuspend":         27,
    "starting":          22,
    "starting_estacion": 23,
    "holding":           24,
    "completing":        25,
}

MOMENTARY_BUTTONS = {"unabort", "unsuspend", "starting", "starting_estacion"}
TOGGLE_BUTTONS    = {"holding", "completing"}

GPIO_DEBOUNCE_S  = 0.20
PULSE_DURATION_S = 0.08

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in BUTTON_PINS.values():
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

button_prev_state = {name: GPIO.input(pin) for name, pin in BUTTON_PINS.items()}
button_last_time  = {name: 0.0 for name in BUTTON_PINS}
active_pulses     = {}
toggle_states = {
    "holding":    False,
    "completing": False,
    "starting_plc": False,
}

# ─── MQTT ─────────────────────────────────────────────────────────────────────
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()

def publish_json(payload: dict):
    msg = json.dumps(payload)
    mqtt_client.publish(TOPIC_PUB, msg, qos=0, retain=False)
    print(f"[MQTT] {msg}")

def publish_startup_message():
    publish_json(PAYLOAD_ZZZ)
    publish_scora_start()
    time.sleep(2)

    publish_scora_home()
    time.sleep(10)

    publish_scora_first()
    time.sleep(15)
def publish_starting_message():
    publish_json(PAYLOAD_ZZZ)

def publish_unabort_unsuspend_message():
    publish_json(PAYLOAD_ZZZ)

def publish_holding_true_message():
    publish_json(PAYLOAD_HOLDING_TRUE)

def publish_completing_true_message():
    publish_json(PAYLOAD_COMPLETING_TRUE)

def publish_toggle_false_message():
    publish_json(PAYLOAD_ZZZ)

def publish_aborted_or_suspended_message():
    publish_json(PAYLOAD_STOPPED)

def publish_end_animation_message():
    publish_json(PAYLOAD_END_ANIMATION)

def publish_current_state():
    if machine_state == "held":
        publish_holding_true_message()

    elif machine_state == "complete":
        publish_completing_true_message()

    elif machine_state == "stopped":
        publish_aborted_or_suspended_message()

    else:
        publish_end_animation_message()  # idle → zzz


def publish_scora_run():
    mqtt_client.publish(TOPIC_RUN, "{}", qos=0, retain=False)
    print("[MQTT] {} -> scora/run")

def publish_scora_first():
    mqtt_client.publish(TOPIC_FIRST, "{}", qos=0, retain=False)
    print("[MQTT] {} -> scora/first")

def publish_scora_start():
    mqtt_client.publish(TOPIC_START, "{}", qos=0, retain=False)
    print("[MQTT] {} -> scora/start")

def publish_scora_stop():
    mqtt_client.publish(TOPIC_STOP, "stop", qos=0, retain=False)
    print("[MQTT] {} -> scora/stop")

def publish_scora_home():
    mqtt_client.publish(TOPIC_HOME , "{}", qos=0, retain=False)
    print("[MQTT] {} -> scora/home")

# ─── PLC ──────────────────────────────────────────────────────────────────────
plc = snap7.client.Client()
plc.connect(PLC_IP, PLC_RACK, PLC_SLOT)

def read_plc_trigger() -> bool:
    data = plc.db_read(DB_NUM, BYTE_IDX, 1)
    return get_bool(data, 0, BIT_IDX)

def read_plc_bool(db_num: int, byte_idx: int, bit_idx: int) -> bool:
    data = plc.db_read(db_num, byte_idx, 1)
    return get_bool(data, 0, bit_idx)

def write_plc_bool(db_num: int, byte_idx: int, bit_idx: int, value: bool):
    data = plc.db_read(db_num, byte_idx, 1)
    buf = bytearray(data)
    set_bool(buf, 0, bit_idx, value)
    plc.db_write(db_num, byte_idx, bytes(buf))

def safe_write_signal(signal_name: str, value: bool):
    byte_idx, bit_idx = PLC_SIGNALS[signal_name]
    try:
        write_plc_bool(PLC_DB_WRITE, byte_idx, bit_idx, value)
        print(f"[PLC WRITE] {signal_name} -> {value}")
    except Exception:
        try:
            plc.disconnect()
        except Exception:
            pass
        try:
            plc.connect(PLC_IP, PLC_RACK, PLC_SLOT)
            write_plc_bool(PLC_DB_WRITE, byte_idx, bit_idx, value)
            print(f"[PLC WRITE] {signal_name} -> {value} (reconectado)")
        except Exception as e:
            print(f"[PLC WRITE ERROR] {signal_name} -> {value}: {e}")

def safe_read_status_signal(signal_name: str) -> bool:
    db_num, byte_idx, bit_idx = PLC_STATUS_READ[signal_name]
    try:
        return read_plc_bool(db_num, byte_idx, bit_idx)
    except Exception:
        try:
            plc.disconnect()
        except Exception:
            pass
        try:
            plc.connect(PLC_IP, PLC_RACK, PLC_SLOT)
            return read_plc_bool(db_num, byte_idx, bit_idx)
        except Exception as e:
            print(f"[PLC READ ERROR] {signal_name}: {e}")
            return False

prev_status_inputs = {
    "aborted":   False,
    "suspended": False,
}

def process_plc_status_edges():
    global prev_status_inputs

    for name in ("aborted", "suspended", "starting_plc"):
        current_val = safe_read_status_signal(name)
        prev_val    = prev_status_inputs.get(name, False)

        # FLANCO POSITIVO
        if (not prev_val) and current_val:

            if name in ("aborted", "suspended"):
                print(f"[PLC EDGE] {name} -> True")
                publish_aborted_or_suspended_message()
                publish_scora_stop()
                set_machine_state("stopped")
                toggle_states["holding"]    = False
                toggle_states["completing"] = False
                safe_write_signal("holding",    False)
                safe_write_signal("completing", False)

            elif name == "starting_plc":
                print("[PLC EDGE] starting_plc -> True")

                publish_starting_message()

                # reset toggles igual que GPIO
                toggle_states["holding"]    = False
                toggle_states["completing"] = False
                safe_write_signal("holding",    False)
                safe_write_signal("completing", False)
                publish_scora_start()
                time.sleep(2)
               

                # 👇 AQUÍ está el cambio importante
                set_machine_state("idle")

        prev_status_inputs[name] = current_val

def process_gpio_buttons(now: float):
    for name, pin in BUTTON_PINS.items():
        current_state = GPIO.input(pin)
        prev_state    = button_prev_state[name]

        if prev_state == GPIO.HIGH and current_state == GPIO.LOW:
            if (now - button_last_time[name]) >= GPIO_DEBOUNCE_S:
                button_last_time[name] = now

                if name in MOMENTARY_BUTTONS:
                    safe_write_signal(name, True)
                    active_pulses[name] = now + PULSE_DURATION_S
                    print(f"[GPIO] {name} pulse")

                    if name in {"starting", "starting_estacion"}:
                        publish_starting_message()

                    if name == "starting_estacion":
                        toggle_states["holding"]    = False
                        toggle_states["completing"] = False
                        safe_write_signal("holding",    False)
                        safe_write_signal("completing", False)
                        if machine_state in ("held", "complete", "stopped"):
                            set_machine_state("idle")
                        print("[GPIO] starting_estacion -> holding=False, completing=False")

                    elif name == "starting":
                        toggle_states["holding"]    = False
                        toggle_states["completing"] = False
                        safe_write_signal("holding",    False)
                        safe_write_signal("completing", False)
                        if machine_state in ("held", "complete", "stopped"):
                            set_machine_state("idle")

                    elif name in {"unabort", "unsuspend"}:
                        if machine_state == "held":
                            return
                        if machine_state == "complete":
                            return
                        publish_unabort_unsuspend_message()
                        if machine_state == "stopped":
                            set_machine_state("idle")

                elif name in TOGGLE_BUTTONS:

                    if machine_state == "stopped":
                        return

                    if name == "holding" and toggle_states["completing"]:
                        print("[GPIO] Ignorado holding porque completing está activo")
                        return

                    if name == "completing" and toggle_states["holding"]:
                        print("[GPIO] Ignorado completing porque holding está activo")
                        return
                    toggle_states[name] = not toggle_states[name]
                    safe_write_signal(name, toggle_states[name])
                    print(f"[GPIO] {name} toggle -> {toggle_states[name]}")

                    if name == "holding":
                        if toggle_states[name]:
                            publish_holding_true_message()
                            set_machine_state("held")
                        else:
                            publish_toggle_false_message()
                            set_machine_state("idle")

                    elif name == "completing":
                        if toggle_states[name]:
                            publish_completing_true_message()
                            set_machine_state("complete")
                        else:
                            publish_toggle_false_message()
                            set_machine_state("idle")

        button_prev_state[name] = current_state

    for name in list(active_pulses.keys()):
        if now >= active_pulses[name]:
            safe_write_signal(name, False)
            del active_pulses[name]



# ─── Estado global ────────────────────────────────────────────────────────────
sticker_status     = "waiting"
status             = "waiting"
color              = (128, 128, 128)
sticker_position   = -100
target_sticker_pos = -100
sticker_angle      = 0
application_phase  = "waiting"
animation_speed    = 18

activation_count  = 0
cycle_active      = False
cycle_end_time    = 0.0
last_trigger_time = 0.0

good_count = 0
bad_count  = 0

execute_start_time = None
execute_total_s    = 0.0
held_start_time    = None
held_total_s       = 0.0
machine_state      = "idle"  # idle | execute | held | complete | stopped

auto_mode      = False
auto_interval  = 2.5
last_auto_time = 0.0

virtual_plc_bit = False
prev_trigger    = False
last_poll_time  = 0.0
plc_trigger     = False

# ─── Helpers ──────────────────────────────────────────────────────────────────
def fmt_time(s: float) -> str:
    m  = int(s) // 60
    ss = int(s) % 60
    ms = int((s - int(s)) * 10)
    return f"{m:02d}:{ss:02d}.{ms}"

def payload_with_counters(base: dict) -> dict:
    p = dict(base)
    p["good"]   = good_count
    p["reject"] = bad_count
    return p

def set_machine_state(new: str):
    """Directly set machine state, handling timer transitions."""
    global machine_state, execute_start_time, execute_total_s
    global held_start_time, held_total_s
    if new == machine_state:
        return
    now = time.time()
    if machine_state == "execute" and execute_start_time:
        execute_total_s   += now - execute_start_time
        execute_start_time = None
    if machine_state == "held" and held_start_time:
        held_total_s   += now - held_start_time
        held_start_time = None
    if new == "execute":
        execute_start_time = now
    elif new == "held":
        held_start_time = now
    machine_state = new

def trigger_cycle():
    global activation_count, cycle_active, cycle_end_time
    global sticker_status, status, color, good_count, bad_count
    global sticker_position, target_sticker_pos, application_phase

    activation_count += 1
    cycle_active = True
    cycle_end_time = time.time() + CYCLE_DURATION_S
    sticker_position = -100; target_sticker_pos = 180; application_phase = "approaching"

    if machine_state in ("stopped", "complete"):
        # Machine not runnable — count as bad, publish feed=1 so DB records the attempt
        bad_count += 1
        sticker_status = "crooked"
        status = "error"
        color  = (200, 0, 0)
        payload = payload_with_counters(PAYLOAD_ERR)
        publish_json(payload)
    elif machine_state == "held":
        # Attempt while held → bad, publish feed=1, stay in held
        bad_count += 1
        sticker_status = "crooked"
        status = "error"
        color  = (200, 0, 0)
        payload = payload_with_counters(PAYLOAD_ERR)
        publish_json(payload)
    else:
        # Normal execution → good, publish run=1
        good_count += 1
        sticker_status = "straight"
        status = "normal"
        color  = (0, 200, 0)
        payload = payload_with_counters(PAYLOAD_OK)
        publish_json(payload)
        set_machine_state("execute")

def stop_cycle():
    global cycle_active, sticker_status, status, color
    global application_phase, sticker_position, target_sticker_pos

    cycle_active = False; sticker_status = "waiting"; status = "waiting"; color = (128, 128, 128)
    application_phase = "waiting"; sticker_position = -100; target_sticker_pos = -100

    publish_current_state()
    publish_scora_run()
    if machine_state not in ("held", "complete", "stopped"):
        set_machine_state("idle")
    set_machine_state("idle")

def reset_all():
    global good_count, bad_count, execute_total_s, held_total_s
    global execute_start_time, held_start_time, activation_count
    global sticker_position, target_sticker_pos, sticker_status, cycle_active
    global application_phase, status, color, sticker_angle, toggle_states

    good_count = bad_count = activation_count = 0
    execute_total_s = held_total_s = 0.0
    execute_start_time = held_start_time = None

    sticker_position   = -100
    target_sticker_pos = -100
    sticker_status     = "waiting"
    cycle_active       = False
    application_phase  = "waiting"
    sticker_angle      = 0
    status             = "waiting"
    color              = (128, 128, 128)
    toggle_states      = {"holding": False, "completing": False}
    set_machine_state("idle")

# ─── Pygame init ──────────────────────────────────────────────────────────────
pygame.init()
screen             = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Sticker Application Station + GPIO PLC Commands")
small_font         = pygame.font.SysFont('Arial', 17)
big_font           = pygame.font.SysFont('Arial', 19, bold=True)
label_font         = pygame.font.SysFont('Arial', 12)
sticker_label_font = pygame.font.SysFont('Arial', 12)
sticker_sm_font    = pygame.font.SysFont('Arial', 10)
clock              = pygame.time.Clock()

STATE_COLORS = {
    "execute":  (0,   210,  80),
    "held":     (255, 160,   0),
    "complete": (0,   160, 255),
    "stopped":  (200,  40,  40),
    "idle":     (100, 100, 120),
}
STATE_LABELS = {
    "execute":  "EXECUTE",
    "held":     "HELD",
    "complete": "COMPLETE",
    "stopped":  "STOPPED",
    "idle":     "IDLE",
}

def rr(surf, col, rect, rad=8, bw=0, bc=None):
    pygame.draw.rect(surf, col, rect, border_radius=rad)
    if bw and bc:
        pygame.draw.rect(surf, bc, rect, bw, border_radius=rad)

def get_bottle_points(x, y, width, height):
    body_height = int(height * 0.75)
    body_y      = y + int(height * 0.25)
    neck_width  = int(width * 0.4)
    neck_height = int(height * 0.25)
    neck_x      = x + (width - neck_width) // 2
    return [
        (neck_x+neck_width, y),      (neck_x+neck_width, body_y),
        (x+width,           body_y), (x+width, body_y+body_height),
        (x,      body_y+body_height),(x,        body_y),
        (neck_x, body_y),            (neck_x,   y),
    ], body_y, body_height, neck_x, neck_width, neck_height

def draw_sticker_applicator(surface, x, y):
    pygame.draw.rect(surface, (120,120,120), (x-50, y-10, 80, 20))
    pygame.draw.rect(surface, (100,100,100), (x-50, y-10, 80, 20), 2)
    pygame.draw.circle(surface, (100,100,100), (x+30, y), 15)
    pygame.draw.circle(surface, (80,80,80),    (x+30, y), 15, 2)
    pygame.draw.rect(surface, (140,140,140), (x-60, y-5, 25, 10))
    pygame.draw.polygon(surface, (200,100,100), [(x-70,y),(x-80,y-5),(x-80,y+5)])

def draw_sticker(surface, x, y, w, h, st, angle=0):
    if st == "missing":
        return
    col = (255, 255, 240)
    if st == "straight":
        pygame.draw.rect(surface, col, (x, y, w, h))
        pygame.draw.rect(surface, (200,200,200), (x, y, w, h), 2)
        t1 = sticker_label_font.render("AGUA",       True, (50, 50, 50))
        t2 = sticker_label_font.render("PURIFICADA", True, (50, 50, 50))
        surface.blit(t1, t1.get_rect(center=(x+w//2, y+h//2-6)))
        surface.blit(t2, t2.get_rect(center=(x+w//2, y+h//2+6)))
    elif st == "crooked":
        cx, cy = x+w//2, y+h//2
        ca, sa = math.cos(angle), math.sin(angle)
        pts = [(cx+dx*ca-dy*sa, cy+dx*sa+dy*ca)
               for dx, dy in [(-w//2,-h//2),(w//2,-h//2),(w//2,h//2),(-w//2,h//2)]]
        pygame.draw.polygon(surface, col, pts)
        pygame.draw.polygon(surface, (200,200,200), pts, 2)
        tm = sticker_sm_font.render("CROOKED", True, (150, 50, 50))
        surface.blit(tm, tm.get_rect(center=(cx, cy)))

# ─── Geometría estática ────────────────────────────────────────────────────────
bottle_x = SIM_W//2 - 60
bottle_y, bottle_w, bottle_h = 160, 120, 300
bpts, body_y, body_h, neck_x, neck_w, neck_h = get_bottle_points(
    bottle_x, bottle_y, bottle_w, bottle_h)

bottle_static = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
pygame.draw.lines(bottle_static, (200,200,200), False, bpts, 3)
pygame.draw.rect(bottle_static, (40,140,255), (bottle_x+3, body_y,    bottle_w-6, body_h))
pygame.draw.rect(bottle_static, (40,140,255), (neck_x+2,   bottle_y,  neck_w-4,   neck_h))
pygame.draw.line(bottle_static, (80,180,255), (neck_x+2, bottle_y), (neck_x+neck_w-2, bottle_y), 2)
cap_y = bottle_y - 15
pygame.draw.rect(bottle_static, (160,160,160), (neck_x-5, cap_y, neck_w+10, 12))
pygame.draw.rect(bottle_static, (120,120,120), (neck_x-5, cap_y, neck_w+10, 12), 2)
ref_y = body_y + int(body_h*0.4)
pygame.draw.line(bottle_static, (100,100,100),
                 (bottle_x-20, ref_y), (bottle_x+bottle_w+20, ref_y), 1)
bottle_static.blit(small_font.render("Alignment Reference", True, (100,100,100)),
                   (bottle_x-20, ref_y-20))
pygame.draw.line(bottle_static, (70,70,100), (SIM_W,0), (SIM_W,HEIGHT), 2)

# Mensaje MQTT al iniciar
publish_startup_message()

running = True

try:
    while running:
        now = time.time()

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif e.key == pygame.K_SPACE:
                    virtual_plc_bit = True
                elif e.key == pygame.K_a:
                    auto_mode      = not auto_mode
                    last_auto_time = now
                    print(f"[AUTO] {'ON' if auto_mode else 'OFF'}")
                elif e.key == pygame.K_r:
                    reset_all()
                    print("[RESET]")
            elif e.type == pygame.KEYUP:
                if e.key == pygame.K_SPACE:
                    virtual_plc_bit = False

        process_gpio_buttons(now)

        if auto_mode and (now - last_auto_time) >= auto_interval:
            last_auto_time  = now
            virtual_plc_bit = True

        if (now - last_poll_time) >= POLL_S:
            last_poll_time = now
            try:
                plc_trigger = read_plc_trigger()
            except Exception:
                try:
                    plc.disconnect()
                except Exception:
                    pass
                try:
                    plc.connect(PLC_IP, PLC_RACK, PLC_SLOT)
                    plc_trigger = read_plc_trigger()
                except Exception:
                    plc_trigger = False

            process_plc_status_edges()

        effective_trigger = plc_trigger or virtual_plc_bit

        if (not prev_trigger) and effective_trigger and (now - last_trigger_time) > DEBOUNCE_S:
            last_trigger_time = now
            if not cycle_active:
                trigger_cycle()

        prev_trigger = effective_trigger

        if virtual_plc_bit and (now - last_trigger_time) > 0.05:
            virtual_plc_bit = False

        if cycle_active and now >= cycle_end_time:
            stop_cycle()

        # Animación sticker
        if application_phase == "approaching":
            if abs(sticker_position - target_sticker_pos) > 1:
                sticker_position = min(sticker_position + animation_speed, target_sticker_pos)
                if sticker_position >= target_sticker_pos - 5:
                    application_phase = "applying"
            else:
                application_phase = "applied"
        elif application_phase == "applying":
            application_phase = "applied"

        sticker_angle = math.radians(15) if sticker_status == "crooked" else 0

        live_execute = execute_total_s + (now - execute_start_time if execute_start_time else 0)
        live_held    = held_total_s    + (now - held_start_time    if held_start_time    else 0)

        # ── Zona simulación ───────────────────────────────────────────────────
        screen.fill((30, 30, 40))

        sb = pygame.Surface((SIM_W-20, 72), pygame.SRCALPHA)
        sb.fill((40,40,50)); pygame.draw.rect(sb, (100,100,120), sb.get_rect(), 2)
        screen.blit(sb, (10, 10))
        screen.blit(small_font.render(f"Sticker: {sticker_status}", True, (255,255,255)), (20, 20))
        screen.blit(small_font.render(f"Status: {status} | Ciclos: {activation_count}",
                                      True, (255,255,255)), (20, 44))

        screen.blit(bottle_static, (0, 0))

        sticker_y = body_y + int(body_h*0.35)
        sticker_w, sticker_h = 80, 30

        if application_phase in ["approaching", "applying"]:
            draw_sticker_applicator(screen, sticker_position-50, sticker_y)

        if sticker_position > -sticker_w:
            if sticker_status == "missing":
                if sticker_position >= target_sticker_pos - 20:
                    mt = small_font.render("NO LABEL", True, (200,0,0))
                    screen.blit(mt, mt.get_rect(center=(bottle_x+bottle_w//2, sticker_y+sticker_h//2)))
            else:
                draw_sticker(screen, sticker_position, sticker_y, sticker_w, sticker_h,
                             sticker_status, sticker_angle)

        if application_phase in ["approaching", "applying"]:
            prog = min((sticker_position+100) / (target_sticker_pos+100), 1.0)
            pygame.draw.rect(screen, (60,60,70),    (20, 490, 140, 7))
            pygame.draw.rect(screen, (0,150,255),   (20, 490, int(140*prog), 7))
            screen.blit(label_font.render(f"Phase: {application_phase}", True, (200,200,200)),
                        (20, 500))

        ind = (SIM_W//2, bottle_y+bottle_h+44)
        pygame.draw.circle(screen, color,         ind, 25)
        pygame.draw.circle(screen, (255,255,255), ind, 25, 2)
        s_short = ("DONE" if color == (0,150,255) else "OK" if color == (0,200,0)
                   else "FAIL" if color == (200,0,0) else "WAIT")
        ss = small_font.render(s_short, True, (255,255,255))
        screen.blit(ss, ss.get_rect(center=ind))

        ctrl_col = (100,200,100) if auto_mode else (160,160,160)
        screen.blit(label_font.render(
            "[SPC] Trigger  [A] AUTO:"+("ON " if auto_mode else "OFF")+"  [R] Reset  [Q] Salir",
            True, ctrl_col), (10, HEIGHT-16))

        # ── Panel derecho ─────────────────────────────────────────────────────
        px = SIM_W + PP
        pw = PANEL_W - 2*PP

        rr(screen, (25,25,38), pygame.Rect(SIM_W+2, PP, PANEL_W-4, HEIGHT-2*PP),
           rad=10, bw=1, bc=(70,70,100))
        py = PP + 10

        screen.blit(label_font.render("▪ PRODUCCIÓN", True, (140,140,180)), (px, py))
        pygame.draw.line(screen, (70,70,100), (px, py+15), (px+pw, py+15), 1)
        py += 23

        cw = (pw-8)//2; rh = 56
        gr = pygame.Rect(px, py, cw, rh)
        rr(screen, (20,60,30), gr, rad=7, bw=1, bc=(0,180,60))
        screen.blit(label_font.render("BUENAS", True, (0,200,80)), (gr.x+6, gr.y+5))
        gv = big_font.render(str(good_count), True, (0,255,100))
        screen.blit(gv, gv.get_rect(centerx=gr.centerx, y=gr.y+24))

        br = pygame.Rect(px+cw+8, py, cw, rh)
        rr(screen, (60,20,20), br, rad=7, bw=1, bc=(200,60,60))
        screen.blit(label_font.render("MALAS", True, (220,80,80)), (br.x+6, br.y+5))
        bv = big_font.render(str(bad_count), True, (255,100,100))
        screen.blit(bv, bv.get_rect(centerx=br.centerx, y=br.y+24))

        py += rh + 12

        screen.blit(label_font.render("▪ TIEMPOS ACUMULADOS", True, (140,140,180)), (px, py))
        pygame.draw.line(screen, (70,70,100), (px, py+15), (px+pw, py+15), 1)
        py += 23

        for lbl, act, val, ac, bc2, dc in [
            ("⏱ T. EXECUTE", machine_state=="execute", live_execute,
             (20,50,30), (0,210,80),  (0,255,80)),
            ("⏸ T. HELD",    machine_state=="held",    live_held,
             (55,35,10), (255,160,0), (255,140,0)),
        ]:
            tr = pygame.Rect(px, py, pw, 54)
            rr(screen, ac if act else (28,32,28), tr, rad=7,
               bw=1, bc=bc2 if act else (50,60,50))
            screen.blit(label_font.render(lbl, True, bc2 if act else (80,100,80)),
                        (tr.x+8, tr.y+6))
            tv = big_font.render(fmt_time(val), True, bc2 if act else (80,110,80))
            screen.blit(tv, tv.get_rect(centerx=tr.centerx, y=tr.y+26))
            if act and int(now*2)%2 == 0:
                pygame.draw.circle(screen, dc, (tr.right-12, tr.y+12), 5)
            py += 54 + 8

        screen.blit(label_font.render("▪ ESTADO ISA-88", True, (140,140,180)), (px, py))
        pygame.draw.line(screen, (70,70,100), (px, py+15), (px+pw, py+15), 1)
        py += 23

        sc  = STATE_COLORS.get(machine_state, (100,100,120))
        sl  = STATE_LABELS.get(machine_state, "IDLE")
        sr  = pygame.Rect(px, py, pw, 54)
        bgs = pygame.Surface((pw, 54), pygame.SRCALPHA)
        r2, g2, b2 = sc
        bgs.fill((r2//4, g2//4, b2//4, 255))
        screen.blit(bgs, (sr.x, sr.y))
        rr(screen, (0,0,0), sr, rad=7, bw=2, bc=sc)
        pygame.draw.circle(screen, sc, (sr.x+16, sr.centery), 8)
        sv2 = big_font.render(sl, True, sc)
        screen.blit(sv2, sv2.get_rect(centery=sr.centery, x=sr.x+32))
        py += 54 + 6

        desc = {
            "execute":  "Aplicando sticker (run=1)",
            "held":     "Pausa por GPIO (feed=1)",
            "complete": "Completado por GPIO (done=1)",
            "stopped":  "Abortado/Suspendido (stopped=1)",
            "idle":     "Esperando trigger",
        }
        dv = label_font.render(desc.get(machine_state, ""), True, (160,160,190))
        screen.blit(dv, dv.get_rect(centerx=px+pw//2, y=py))

        pygame.display.flip()
        clock.tick(25)

finally:
    pygame.quit()
    publish_json(PAYLOAD_OFF)
    publish_scora_start()
    time.sleep(10)
    publish_scora_home()
    time.sleep(15)
    publish_scora_stop()
    time.sleep(10)
    try:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    except Exception:
        pass
    try:
        plc.disconnect()
    except Exception:
        pass
    try:
        GPIO.cleanup()
    except Exception:
        pass
