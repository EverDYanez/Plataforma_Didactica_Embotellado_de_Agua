"""
cappingsim_gpio_plc.py  –  Estación de capsulado de botellas
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
  A        → modo automático (trigger cada ~2 s)
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
WIDTH, HEIGHT = 650, 550
BOTTLE_W_AREA = 400

BROKER    = "192.168.1.2"
PORT      = 1883
TOPIC_PUB = "capping/status"

PLC_IP   = "192.168.1.33"
PLC_RACK = 0
PLC_SLOT = 1

# Trigger de ciclo leído del PLC
DB_NUM   = 1
BYTE_IDX = 3
BIT_IDX  = 0

# DB donde se escriben los comandos desde GPIO
PLC_DB_WRITE = 1

PLC_SIGNALS = {
    "unabort":           (0, 0),
    "unsuspend":         (1, 7),
    "starting":          (1, 2),
    "starting_estacion": (1, 5),
    "holding":           (0, 2),
    "completing":        (0, 6),
}

# Ajusta estos bits si aborted y suspended están en otra dirección real
PLC_STATUS_READ = {
    "aborted":   (1, 2, 3),  # (db_num, byte_idx, bit_idx)
    "suspended": (1, 2, 4),
    "starting_plc": (1, 2, 5),  # AJUSTA si es otra dirección real
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
torque_value    = None
target_torque   = 2.5
current_torque  = 0.0
status          = "waiting"
color           = (128, 128, 128)
cap_rotation    = 0.0
capping_active  = False
animation_speed = 0.2

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
auto_interval  = 2.0
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
    global capping_active, status, color, torque_value, good_count, bad_count

    activation_count += 1
    cycle_active   = True
    cycle_end_time = time.time() + CYCLE_DURATION_S
    capping_active = True
    torque_value   = 2.5

    if machine_state in ("stopped", "complete"):
        # Machine not runnable — count as bad, publish feed=1 so DB records the attempt
        bad_count += 1
        status = "error"
        color  = (200, 0, 0)
        payload = payload_with_counters(PAYLOAD_ERR)
        publish_json(payload)
    elif machine_state == "held":
        # Attempt while held → bad, publish feed=1, stay in held
        bad_count += 1
        status = "error"
        color  = (200, 0, 0)
        payload = payload_with_counters(PAYLOAD_ERR)
        publish_json(payload)
    else:
        # Normal execution → good, publish run=1
        good_count += 1
        status = "normal"
        color  = (0, 200, 0)
        payload = payload_with_counters(PAYLOAD_OK)
        publish_json(payload)
        set_machine_state("execute")

def stop_cycle():
    global cycle_active, capping_active, torque_value, status, color

    cycle_active   = False
    capping_active = False
    torque_value   = None
    status = "waiting"
    color  = (128, 128, 128)

    publish_current_state()
    if machine_state not in ("held", "complete", "stopped"):
        set_machine_state("idle")

def reset_all():
    global good_count, bad_count, execute_total_s, held_total_s
    global execute_start_time, held_start_time, activation_count
    global current_torque, torque_value, cycle_active
    global capping_active, status, color, cap_rotation
    global toggle_states

    good_count = 0
    bad_count  = 0
    activation_count   = 0
    execute_total_s    = 0.0
    held_total_s       = 0.0
    execute_start_time = None
    held_start_time    = None

    current_torque  = 0.0
    torque_value    = None
    cycle_active    = False
    capping_active  = False
    cap_rotation    = 0.0
    status          = "waiting"
    color           = (128, 128, 128)
    toggle_states   = {"holding": False, "completing": False}
    set_machine_state("idle")

# ─── Pygame init ──────────────────────────────────────────────────────────────
pygame.init()
screen     = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Capping Station + GPIO PLC Commands")
small_font = pygame.font.SysFont('Arial', 18)
big_font   = pygame.font.SysFont('Arial', 22, bold=True)
label_font = pygame.font.SysFont('Arial', 13)
clock      = pygame.time.Clock()

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

def draw_rounded_rect(surf, col, rect, radius=10, border=0, border_color=None):
    pygame.draw.rect(surf, col, rect, border_radius=radius)
    if border and border_color:
        pygame.draw.rect(surf, border_color, rect, border, border_radius=radius)

def get_bottle_points(x, y, w, h):
    bh  = int(h * 0.75)
    by  = y + int(h * 0.25)
    nw  = int(w * 0.4)
    nx  = x + (w - nw) // 2
    pts = [(nx,y),(nx+nw,y),(nx+nw,by),(x+w,by),(x+w,by+bh),(x,by+bh),(x,by),(nx,by),(nx,y)]
    return pts, by, bh, nx, nw, int(h*0.25)

def draw_capping_mechanism(surf, cx, cy, angle, torque):
    r = 40
    pygame.draw.circle(surf, (150,150,150), (cx, cy), r)
    pygame.draw.circle(surf, (100,100,100), (cx, cy), r, 3)
    for i in range(8):
        a = angle + i * math.pi / 4
        pygame.draw.line(surf, (80,80,80),
            (cx+int((r-15)*math.cos(a)), cy+int((r-15)*math.sin(a))),
            (cx+int((r-5 )*math.cos(a)), cy+int((r-5 )*math.sin(a))), 2)
    pygame.draw.circle(surf, (60,60,60), (cx, cy), 5)
    if torque > 0:
        intensity = min(torque / 5.0, 1.0)
        tc = (int(255*intensity), int(255*(1-intensity)), 0)
        pygame.draw.circle(surf, tc, (cx, cy), r+5, 3)

def draw_cap(surf, x, y, w, angle, applied=False):
    col = (160,160,160) if applied else (180,180,180)
    ch  = 12
    pygame.draw.rect(surf, col, (x, y, w, ch))
    pygame.draw.rect(surf, (120,120,120), (x, y, w, ch), 2)
    for i in range(8):
        a  = angle + i * 2 * math.pi / 8
        rx = x + w//2 + int(8 * math.cos(a))
        ry = y + ch//2 + int(3 * math.sin(a))
        if x < rx < x+w:
            pygame.draw.circle(surf, (140,140,140), (rx, ry), 2)

# Pre-calcular elementos estáticos
bottle_x, bottle_y, bottle_w, bottle_h = BOTTLE_W_AREA//2-60, 225, 120, 280
bpts, body_y, body_h, neck_x, neck_w, neck_h = get_bottle_points(
    bottle_x, bottle_y, bottle_w, bottle_h)

scale_x, scale_start_y, scale_height = 60, 175, 300

bottle_static = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
pygame.draw.lines(bottle_static, (200,200,200), False, bpts, 3)
pygame.draw.rect(bottle_static, (40,140,255), (bottle_x+3, body_y, bottle_w-6, body_h))
pygame.draw.rect(bottle_static, (40,140,255), (neck_x+2, bottle_y, neck_w-4, neck_h))
pygame.draw.line(bottle_static, (80,180,255), (neck_x+2, bottle_y), (neck_x+neck_w-2, bottle_y), 2)
for i, t in enumerate([6, 5, 4, 3, 2, 1, 0]):
    my = scale_start_y + i * scale_height // 6
    pygame.draw.line(bottle_static, (150,150,150), (scale_x-10, my), (scale_x, my), 2)
    bottle_static.blit(small_font.render(str(t), True, (150,150,150)), (scale_x-25, my-8))
pygame.draw.rect(bottle_static, (60,60,70), (scale_x-5, scale_start_y, 10, scale_height), 2)
rt = pygame.transform.rotate(small_font.render("Torque (Nm)", True, (150,150,150)), 90)
bottle_static.blit(rt, (scale_x-55, scale_start_y + scale_height//2 - 40))

PANEL_LEFT = BOTTLE_W_AREA + 5
PANEL_PAD  = 8

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

        # Animación de torque
        target_t = 2.5 if capping_active else 0.0
        diff = target_t - current_torque
        if abs(diff) > 0.05:
            current_torque += animation_speed * (1 if diff > 0 else -1)
            current_torque  = max(0.0, current_torque)

        if capping_active and current_torque > 0:
            cap_rotation += current_torque * 0.1
            if cap_rotation > 2*math.pi:
                cap_rotation = 0

        live_execute = execute_total_s + (now - execute_start_time if execute_start_time else 0)
        live_held    = held_total_s    + (now - held_start_time    if held_start_time    else 0)

        # ── Dibujar ───────────────────────────────────────────────────────────
        screen.fill((30, 30, 40))

        sb = pygame.Surface((BOTTLE_W_AREA-10, 80), pygame.SRCALPHA)
        sb.fill((40,40,50))
        pygame.draw.rect(sb, (100,100,120), sb.get_rect(), 2)
        screen.blit(sb, (10, 10))

        screen.blit(bottle_static, (0, 0))

        screen.blit(small_font.render(
            f"Torque: {torque_value if torque_value is not None else '--'} Nm",
            True, (255,255,255)), (20, 25))
        screen.blit(small_font.render(
            f"Status: {status} | Ciclos: {activation_count}",
            True, (255,255,255)), (20, 50))

        # Barra de torque vertical
        if current_torque > 0:
            bh2 = int((current_torque / 6.0) * scale_height)
            bc  = (0,200,0) if 2 <= current_torque <= 5 else (255,100,100)
            by2 = scale_start_y + scale_height - bh2
            pygame.draw.rect(screen, bc, (scale_x-3, by2, 6, bh2))
            pygame.draw.circle(screen, bc, (scale_x, by2-10), 8)
            screen.blit(small_font.render(f"{current_torque:.1f}", True, (255,255,255)),
                        (scale_x+15, by2-18))

        cap_y = bottle_y - 15
        draw_cap(screen, neck_x-5, cap_y, neck_w+10, cap_rotation, current_torque > 1.0)
        mechanism_y = bottle_y - 80
        draw_capping_mechanism(screen, BOTTLE_W_AREA//2, mechanism_y, cap_rotation, current_torque)

        if capping_active:
            pygame.draw.line(screen, (120,120,120),
                             (BOTTLE_W_AREA//2, mechanism_y+40), (BOTTLE_W_AREA//2, cap_y), 4)
            for r in [pygame.Rect(BOTTLE_W_AREA//2-8,  mechanism_y+45, 16, 20),
                      pygame.Rect(BOTTLE_W_AREA//2-12, mechanism_y+65, 24, 15)]:
                pygame.draw.rect(screen, (100,100,100), r)
                pygame.draw.rect(screen, (80,80,80),    r, 2)

        ind_x = BOTTLE_W_AREA - 35
        pygame.draw.circle(screen, color,         (ind_x, 50), 25)
        pygame.draw.circle(screen, (255,255,255), (ind_x, 50), 25, 2)
        if status == "done":     short = "DONE"
        elif color == (0,200,0): short = "OK"
        elif color == (200,0,0): short = "FAIL"
        else:                    short = "WAIT"
        ss = small_font.render(short, True, (255,255,255))
        screen.blit(ss, ss.get_rect(center=(ind_x, 50)))

        pygame.draw.line(screen, (70,70,100), (BOTTLE_W_AREA,0), (BOTTLE_W_AREA,HEIGHT), 2)

        ctrl_col = (100,200,100) if auto_mode else (160,160,160)
        screen.blit(label_font.render(
            f"[SPC] Trigger  [A] {'AUTO:ON' if auto_mode else 'AUTO:OFF'}  [R] Reset  [Q] Salir",
            True, ctrl_col), (10, HEIGHT-18))

        # ── Panel derecho ─────────────────────────────────────────────────────
        panel_w    = WIDTH - PANEL_LEFT - PANEL_PAD
        panel_rect = pygame.Rect(PANEL_LEFT, PANEL_PAD, panel_w, HEIGHT-2*PANEL_PAD)
        draw_rounded_rect(screen, (25,25,38), panel_rect, radius=12, border=1, border_color=(70,70,100))

        px = PANEL_LEFT + PANEL_PAD
        pw = panel_w - 2*PANEL_PAD
        py = PANEL_PAD + 10

        screen.blit(label_font.render("▪ MÉTRICAS", True, (140,140,180)), (px+2, py))
        pygame.draw.line(screen, (70,70,100), (px, py+16), (px+pw, py+16), 1)
        py += 24

        row_h = 62
        gr = pygame.Rect(px, py, pw, row_h)
        draw_rounded_rect(screen, (20,60,30), gr, radius=8, border=1, border_color=(0,180,60))
        screen.blit(label_font.render("BOTELLAS BUENAS", True, (0,200,80)), (gr.x+8, gr.y+5))
        gv = big_font.render(str(good_count), True, (0,255,100))
        screen.blit(gv, gv.get_rect(centerx=gr.centerx, y=gr.y+26))

        py += row_h + 6
        br = pygame.Rect(px, py, pw, row_h)
        draw_rounded_rect(screen, (60,20,20), br, radius=8, border=1, border_color=(200,60,60))
        screen.blit(label_font.render("BOTELLAS MALAS", True, (220,80,80)), (br.x+8, br.y+5))
        bv = big_font.render(str(bad_count), True, (255,100,100))
        screen.blit(bv, bv.get_rect(centerx=br.centerx, y=br.y+26))

        py += row_h + 12

        screen.blit(label_font.render("▪ TIEMPOS", True, (140,140,180)), (px+2, py))
        pygame.draw.line(screen, (70,70,100), (px, py+16), (px+pw, py+16), 1)
        py += 24

        er = pygame.Rect(px, py, pw, row_h)
        ea = (machine_state == "execute")
        draw_rounded_rect(screen, (20,50,30) if ea else (28,38,28), er, radius=8,
                          border=1, border_color=(0,210,80) if ea else (50,100,50))
        screen.blit(label_font.render("⏱ T. EXECUTE", True, (0,210,80) if ea else (80,130,80)),
                    (er.x+8, er.y+5))
        ev = big_font.render(fmt_time(live_execute), True, (0,255,100) if ea else (80,160,80))
        screen.blit(ev, ev.get_rect(centerx=er.centerx, y=er.y+26))
        if ea:
            pygame.draw.circle(screen, (0,255,80), (er.right-10, er.y+10), 5)

        py += row_h + 6
        hr = pygame.Rect(px, py, pw, row_h)
        ha = (machine_state == "held")
        draw_rounded_rect(screen, (55,35,10) if ha else (35,30,22), hr, radius=8,
                          border=1, border_color=(255,160,0) if ha else (100,80,30))
        screen.blit(label_font.render("⏸ T. HELD", True, (255,160,0) if ha else (130,100,40)),
                    (hr.x+8, hr.y+5))
        hv = big_font.render(fmt_time(live_held), True, (255,200,0) if ha else (130,100,40))
        screen.blit(hv, hv.get_rect(centerx=hr.centerx, y=hr.y+26))
        if ha and int(now*2)%2 == 0:
            pygame.draw.circle(screen, (255,140,0), (hr.right-10, hr.y+10), 5)

        py += row_h + 12

        screen.blit(label_font.render("▪ ESTADO ISA-88", True, (140,140,180)), (px+2, py))
        pygame.draw.line(screen, (70,70,100), (px, py+16), (px+pw, py+16), 1)
        py += 24

        sc = STATE_COLORS.get(machine_state, (100,100,120))
        sl = STATE_LABELS.get(machine_state, "IDLE")
        sr = pygame.Rect(px, py, pw, 52)
        bg_s = pygame.Surface((sr.width, sr.height), pygame.SRCALPHA)
        r2, g2, b2 = sc
        bg_s.fill((r2//4, g2//4, b2//4, 255))
        screen.blit(bg_s, (sr.x, sr.y))
        draw_rounded_rect(screen, (0,0,0), sr, radius=8, border=2, border_color=sc)
        pygame.draw.circle(screen, sc, (sr.x+16, sr.centery), 8)
        sv2 = big_font.render(sl, True, sc)
        screen.blit(sv2, sv2.get_rect(centery=sr.centery, x=sr.x+32))

        py += 58
        desc = {
            "execute":  "Proceso activo\n(run=1)",
            "held":     "Pausa por GPIO\n(feed=1)",
            "complete": "Completado por GPIO\n(done=1)",
            "stopped":  "Abortado/Suspendido\n(stopped=1)",
            "idle":     "Esperando\ntrigger",
        }
        for line in desc.get(machine_state, "").split("\n"):
            dl = label_font.render(line, True, (180,180,200))
            screen.blit(dl, dl.get_rect(centerx=px+pw//2, y=py))
            py += 16

        pygame.display.flip()
        clock.tick(25)

finally:
    pygame.quit()
    publish_json(PAYLOAD_OFF)
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
