"""
washingsim_gpio_plc.py  –  Estación de lavado de botellas
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
import random
import threading
import queue
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import psutil

import snap7
from snap7.util import get_bool, set_bool

# ─── Config ───────────────────────────────────────────────────────────────────
WIDTH, HEIGHT = 650, 550
BOTTLE_W_AREA = 400

msg_counter = 0

BROKER    = "192.168.1.2"
PORT      = 1883
TOPIC_PUB = "washing/status"
PLC_IP    = "192.168.1.33"
PLC_RACK  = 0
PLC_SLOT  = 1

# Todos los bits de estado están en DB 1, byte 2.
# Se lee el byte completo en una sola llamada y se extraen los bits.
DB_STATUS    = 1
BYTE_STATUS  = 2
BIT_TRIGGER  = 6   # trigger de ciclo
BIT_ABORTED  = 3
BIT_SUSPENDED = 4
BIT_STARTING_PLC = 5

# DB y señales para escrituras desde GPIO
PLC_DB_WRITE = 1
PLC_SIGNALS = {
    "unabort":           (0, 0),
    "unsuspend":         (1, 7),
    "starting":          (1, 2),
    "starting_estacion": (1, 3),
    "holding":           (0, 4),
    "completing":        (1, 0),
}

POLL_S           = 0.02
DEBOUNCE_S       = 0.35
CYCLE_DURATION_S = 2.0

PAYLOAD_OK              = {"run":1,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":0,"speed":60, "good":0,"reject":0}
PAYLOAD_ERR             = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":1,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_OFF             = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_ZZZ             = {"run":0,"fault":0,"stopped":0,"zzz":1,"feed":0,"done":0,"speed":60, "good":0,"reject":0}
PAYLOAD_HOLDING_TRUE    = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":1,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_COMPLETING_TRUE = {"run":0,"fault":0,"stopped":0,"zzz":0,"feed":0,"done":1,"speed":0.4,"good":0,"reject":0}
PAYLOAD_STOPPED         = {"run":0,"fault":0,"stopped":1,"zzz":0,"feed":0,"done":0,"speed":0.4,"good":0,"reject":0}
PAYLOAD_END_ANIMATION   = {"run":0,"fault":0,"stopped":0,"zzz":1,"feed":0,"done":0,"speed":60, "good":0,"reject":0}

# ─── GPIO (BCM) ───────────────────────────────────────────────────────────────
BUTTON_PINS = {
    "unabort":           17,
    "unsuspend":         27,
    "starting":          22,
    "starting_estacion": 23,
    "holding":           16,
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
    "holding":      False,
    "completing":   False,
    "starting_plc": False,
}

process = psutil.Process()  # Para monitorear uso de CPU y memoria
last_sys_metrics = 0
cpu_percent_cached = 0
ram_percent_cached = 0

# ─── MQTT ─────────────────────────────────────────────────────────────────────
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()

def publish_json(payload: dict, ts_override: float = None):
    """
    Publica payload JSON por MQTT.
    ts_override: timestamp en ms capturado en el momento del evento.
                 Si se omite se captura aquí (menos preciso).
    """
    snapshot = get_plc_snapshot()
    global msg_counter
    msg_counter += 1
    p = dict(payload)
    p["ts_envio"] = ts_override if ts_override is not None else time.time() * 1000
    p["msg_id"]   = msg_counter
    p["plc_read_latency"] = round(snapshot["latency_read"],3)
    global last_sys_metrics
    global cpu_percent_cached
    global ram_percent_cached

    now = time.time()

    if now - last_sys_metrics >= 1.0:
        cpu_percent_cached = process.cpu_percent(interval=None)/psutil.cpu_count()
        ram_percent_cached = process.memory_percent()
        last_sys_metrics = now
    p["cpu_percent"] = cpu_percent_cached
    p["memory_percent"] = ram_percent_cached
    mqtt_client.publish(TOPIC_PUB, json.dumps(p), qos=0, retain=False)
    print(f"[MQTT] {p}")

# ─── Snap7: hilo de fondo ────────────────────────────────────────────────────
# Valores leídos del PLC, compartidos con el hilo principal mediante Lock.
_plc_lock = threading.Lock()
_plc_vals = {
    "trigger":      False,
    "aborted":      False,
    "suspended":    False,
    "starting_plc": False,
}

# Cola para escrituras al PLC solicitadas desde el hilo principal.
# Cada item es (byte_idx, bit_idx, value: bool)
_write_queue: queue.Queue = queue.Queue()

_snap7_running = True

def _snap7_thread_fn():
    """
    Hilo daemon que:
      - Lee 1 byte del PLC cada POLL_S (4 bits de estado en 1 round-trip).
      - Procesa la cola de escrituras pendientes.
    El hilo principal nunca se bloquea en operaciones Snap7.
    """
    plc_bg = snap7.client.Client()

    def _connect():
        for _ in range(5):
            try:
                plc_bg.connect(PLC_IP, PLC_RACK, PLC_SLOT)
                if plc_bg.get_connected():
                    return True
            except Exception:
                pass
            time.sleep(0.5)
        return False

    _connect()

    while _snap7_running:
        t0 = time.perf_counter()

        # ── 1. Escrituras pendientes ─────────────────────────────────────────
        while not _write_queue.empty():
            try:
                byte_idx, bit_idx, value = _write_queue.get_nowait()
                try:
                    data = plc_bg.db_read(PLC_DB_WRITE, byte_idx, 1)
                    buf  = bytearray(data)
                    set_bool(buf, 0, bit_idx, value)
                    plc_bg.db_write(PLC_DB_WRITE, byte_idx, bytes(buf))
                except Exception:
                    if not plc_bg.get_connected():
                        _connect()
            except queue.Empty:
                break

        # ── 2. Lectura de estado: 1 byte, 4 bits, 1 round-trip ──────────────
        try:
            if not plc_bg.get_connected():
                _connect()
            read_t0 = time.perf_counter()
            data = plc_bg.db_read(DB_STATUS, BYTE_STATUS, 1)
            trigger      = get_bool(data, 0, BIT_TRIGGER)
            aborted      = get_bool(data, 0, BIT_ABORTED)
            suspended    = get_bool(data, 0, BIT_SUSPENDED)
            starting_plc = get_bool(data, 0, BIT_STARTING_PLC)
            read_latency_ms = (time.perf_counter()-read_t0)*1000
            with _plc_lock:
                _plc_vals["trigger"]      = trigger
                _plc_vals["aborted"]      = aborted
                _plc_vals["suspended"]    = suspended
                _plc_vals["starting_plc"] = starting_plc
                _plc_vals["latency_read"] = read_latency_ms
        except Exception:
            if not plc_bg.get_connected():
                _connect()

        # ── 3. Esperar el resto del intervalo de polling ─────────────────────
        elapsed = time.perf_counter() - t0
        sleep_t = POLL_S - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    try:
        plc_bg.disconnect()
    except Exception:
        pass


snap7_thread = threading.Thread(target=_snap7_thread_fn, daemon=True, name="snap7-bg")
snap7_thread.start()


def enqueue_write(signal_name: str, value: bool):
    """Encola una escritura al PLC sin bloquear el hilo principal."""
    byte_idx, bit_idx = PLC_SIGNALS[signal_name]
    _write_queue.put((byte_idx, bit_idx, value))
    print(f"[PLC WRITE enqueued] {signal_name} -> {value}")


def get_plc_snapshot():
    """Devuelve una copia de los valores leídos por el hilo de Snap7."""
    with _plc_lock:
        return dict(_plc_vals)

# ─── Estado global ────────────────────────────────────────────────────────────
pressure_value   = None
target_pressure  = 0.0
current_pressure = 0.0
status           = "waiting"
color            = (128, 128, 128)
wash_phase       = 0.0
washing_active   = False
animation_speed  = 0.2

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
plc_trigger     = False

prev_status_inputs = {
    "aborted":      False,
    "suspended":    False,
    "starting_plc": False,
}

# ─── Partículas de agua ───────────────────────────────────────────────────────
class WaterDrop:
    def __init__(self, bx, by, bw):
        self.x     = bx + random.randint(10, bw - 10)
        self.y     = by + random.randint(0, 20)
        self.vy    = random.uniform(1.5, 3.5)
        self.vx    = random.uniform(-0.4, 0.4)
        self.r     = random.randint(2, 5)
        self.life  = 1.0
        self.decay = random.uniform(0.02, 0.05)
    def update(self):
        self.y   += self.vy
        self.x   += self.vx
        self.life -= self.decay
    @property
    def alive(self):
        return self.life > 0

class SprayJet:
    def __init__(self, cx, top_y):
        angle      = random.uniform(-0.3, 0.3)
        speed      = random.uniform(4, 7)
        self.x     = cx + random.randint(-15, 15)
        self.y     = top_y
        self.vx    = math.sin(angle) * speed
        self.vy    = speed * math.cos(angle)
        self.r     = random.randint(2, 4)
        self.life  = 1.0
        self.decay = random.uniform(0.03, 0.07)
    def update(self):
        self.y   += self.vy
        self.x   += self.vx
        self.vy  += 0.15
        self.life -= self.decay
    @property
    def alive(self):
        return self.life > 0

water_drops = []
spray_jets  = []

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
    global washing_active, status, color, pressure_value, good_count, bad_count, wash_phase

    # ── Timestamp capturado ANTES de cualquier otra operación ────────────────
    ts = time.time() * 1000

    activation_count += 1
    cycle_active   = True
    cycle_end_time = time.time() + CYCLE_DURATION_S
    washing_active = True
    wash_phase     = 0.0
    pressure_value = 3.2

    if machine_state in ("stopped", "complete"):
        bad_count += 1
        status = "error"
        color  = (200, 0, 0)
        publish_json(payload_with_counters(PAYLOAD_ERR), ts_override=ts)
    elif machine_state == "held":
        bad_count += 1
        status = "error"
        color  = (200, 0, 0)
        publish_json(payload_with_counters(PAYLOAD_ERR), ts_override=ts)
    else:
        good_count += 1
        status = "normal"
        color  = (0, 200, 0)
        publish_json(payload_with_counters(PAYLOAD_OK), ts_override=ts)
        set_machine_state("execute")

def stop_cycle():
    global cycle_active, washing_active, pressure_value, status, color

    ts = time.time() * 1000   # timestamp temprano

    cycle_active   = False
    washing_active = False
    pressure_value = None
    status = "waiting"
    color  = (128, 128, 128)

    _publish_current_state(ts)
    if machine_state not in ("held", "complete", "stopped"):
        set_machine_state("idle")
    water_drops.clear()
    spray_jets.clear()

def _publish_current_state(ts: float = None):
    if ts is None:
        ts = time.time() * 1000
    if machine_state == "held":
        publish_json(payload_with_counters(PAYLOAD_HOLDING_TRUE),    ts_override=ts)
    elif machine_state == "complete":
        publish_json(payload_with_counters(PAYLOAD_COMPLETING_TRUE), ts_override=ts)
    elif machine_state == "stopped":
        publish_json(payload_with_counters(PAYLOAD_STOPPED),         ts_override=ts)
    else:
        publish_json(payload_with_counters(PAYLOAD_END_ANIMATION),   ts_override=ts)

def reset_all():
    global good_count, bad_count, execute_total_s, held_total_s
    global execute_start_time, held_start_time, activation_count
    global current_pressure, pressure_value, cycle_active
    global washing_active, status, color, wash_phase, toggle_states

    good_count         = 0
    bad_count          = 0
    activation_count   = 0
    execute_total_s    = 0.0
    held_total_s       = 0.0
    execute_start_time = None
    held_start_time    = None
    current_pressure   = 0.0
    pressure_value     = None
    cycle_active       = False
    washing_active     = False
    wash_phase         = 0.0
    status             = "waiting"
    color              = (128, 128, 128)
    toggle_states      = {"holding": False, "completing": False}
    set_machine_state("idle")
    water_drops.clear()
    spray_jets.clear()

# ─── Procesamiento de flancos de estado del PLC ───────────────────────────────
def process_plc_status_edges(snapshot: dict):
    """
    Recibe el snapshot ya leído por el hilo de Snap7.
    No hace ninguna llamada de red: solo lógica y publicaciones MQTT.
    """
    global prev_status_inputs

    for name in ("aborted", "suspended", "starting_plc"):
        current_val = snapshot.get(name, False)
        prev_val    = prev_status_inputs.get(name, False)

        if (not prev_val) and current_val:
            ts = time.time() * 1000   # timestamp al detectar el flanco

            if name in ("aborted", "suspended"):
                print(f"[PLC EDGE] {name} -> True")
                publish_json(payload_with_counters(PAYLOAD_STOPPED), ts_override=ts)
                set_machine_state("stopped")
                toggle_states["holding"]    = False
                toggle_states["completing"] = False
                enqueue_write("holding",    False)
                enqueue_write("completing", False)

            elif name == "starting_plc":
                print("[PLC EDGE] starting_plc -> True")
                publish_json(payload_with_counters(PAYLOAD_ZZZ), ts_override=ts)
                toggle_states["holding"]    = False
                toggle_states["completing"] = False
                enqueue_write("holding",    False)
                enqueue_write("completing", False)
                set_machine_state("idle")

        prev_status_inputs[name] = current_val

# ─── Procesamiento de botones GPIO ───────────────────────────────────────────
def process_gpio_buttons(now: float):
    for name, pin in BUTTON_PINS.items():
        current_state = GPIO.input(pin)
        prev_state    = button_prev_state[name]

        if prev_state == GPIO.HIGH and current_state == GPIO.LOW:
            if (now - button_last_time[name]) >= GPIO_DEBOUNCE_S:
                button_last_time[name] = now
                ts = now * 1000   # timestamp temprano al detectar el botón

                if name in MOMENTARY_BUTTONS:
                    enqueue_write(name, True)
                    active_pulses[name] = now + PULSE_DURATION_S
                    print(f"[GPIO] {name} pulse")

                    if name in {"starting", "starting_estacion"}:
                        publish_json(payload_with_counters(PAYLOAD_ZZZ), ts_override=ts)

                    if name == "starting_estacion":
                        toggle_states["holding"]    = False
                        toggle_states["completing"] = False
                        enqueue_write("holding",    False)
                        enqueue_write("completing", False)
                        if machine_state in ("held", "complete", "stopped"):
                            set_machine_state("idle")
                        print("[GPIO] starting_estacion -> holding=False, completing=False")

                    elif name == "starting":
                        toggle_states["holding"]    = False
                        toggle_states["completing"] = False
                        enqueue_write("holding",    False)
                        enqueue_write("completing", False)
                        if machine_state in ("held", "complete", "stopped"):
                            set_machine_state("idle")

                    elif name in {"unabort", "unsuspend"}:
                        if machine_state in ("held", "complete"):
                            pass
                        else:
                            publish_json(payload_with_counters(PAYLOAD_ZZZ), ts_override=ts)
                            if machine_state == "stopped":
                                set_machine_state("idle")

                elif name in TOGGLE_BUTTONS:
                    if machine_state == "stopped":
                        pass
                    elif name == "holding" and toggle_states["completing"]:
                        print("[GPIO] Ignorado holding porque completing está activo")
                    elif name == "completing" and toggle_states["holding"]:
                        print("[GPIO] Ignorado completing porque holding está activo")
                    else:
                        toggle_states[name] = not toggle_states[name]
                        enqueue_write(name, toggle_states[name])
                        print(f"[GPIO] {name} toggle -> {toggle_states[name]}")

                        if name == "holding":
                            if toggle_states[name]:
                                publish_json(payload_with_counters(PAYLOAD_HOLDING_TRUE),    ts_override=ts)
                                set_machine_state("held")
                            else:
                                publish_json(payload_with_counters(PAYLOAD_ZZZ),             ts_override=ts)
                                set_machine_state("idle")
                        elif name == "completing":
                            if toggle_states[name]:
                                publish_json(payload_with_counters(PAYLOAD_COMPLETING_TRUE), ts_override=ts)
                                set_machine_state("complete")
                            else:
                                publish_json(payload_with_counters(PAYLOAD_ZZZ),             ts_override=ts)
                                set_machine_state("idle")

        button_prev_state[name] = current_state

    for name in list(active_pulses.keys()):
        if now >= active_pulses[name]:
            enqueue_write(name, False)
            del active_pulses[name]

# ─── Pygame init ──────────────────────────────────────────────────────────────
pygame.init()
screen     = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Washing Station + GPIO PLC Commands")
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

# ─── Geometría botella INVERTIDA ──────────────────────────────────────────────
bottle_x = BOTTLE_W_AREA // 2 - 60
bottle_y = 130
bottle_w = 120
bottle_h = 280

body_h   = int(bottle_h * 0.68)
neck_h   = bottle_h - body_h
neck_w   = int(bottle_w * 0.38)
neck_x   = bottle_x + (bottle_w - neck_w) // 2

body_top    = bottle_y
body_bottom = bottle_y + body_h
neck_top    = body_bottom
neck_bottom = bottle_y + bottle_h
bottle_cx   = bottle_x + bottle_w // 2

def inverted_bottle_points():
    return [
        (bottle_x,          body_top),
        (bottle_x+bottle_w, body_top),
        (bottle_x+bottle_w, body_bottom),
        (neck_x+neck_w,     neck_top),
        (neck_x+neck_w,     neck_bottom),
        (neck_x,            neck_bottom),
        (neck_x,            neck_top),
        (bottle_x,          body_bottom),
        (bottle_x,          body_top),
    ]

bpts = inverted_bottle_points()

scale_x       = 60
scale_start_y = 160
scale_height  = 270

PANEL_LEFT = BOTTLE_W_AREA + 5
PANEL_PAD  = 8

def draw_nozzle(surf, cx, y):
    nw = 20
    nh = 30
    pygame.draw.rect(surf, (160,160,160), (cx-nw//2, y, nw, nh))
    pygame.draw.rect(surf, (100,100,100), (cx-nw//2, y, nw, nh), 2)
    tip_pts = [(cx-8,y+nh),(cx+8,y+nh),(cx+4,y+nh+10),(cx-4,y+nh+10)]
    pygame.draw.polygon(surf, (130,130,130), tip_pts)
    pygame.draw.polygon(surf, (90,90,90),    tip_pts, 2)
    pygame.draw.rect(surf, (120,120,120), (cx-5, y-40, 10, 40))
    pygame.draw.rect(surf, (90,90,90),    (cx-5, y-40, 10, 40), 2)

def draw_wash_level(surf, phase, bx, by_top, bw, bh):
    if phase <= 0:
        return
    fill_h = int(bh * phase)
    fill_y = by_top + (bh - fill_h)
    ws = pygame.Surface((bw - 6, fill_h), pygame.SRCALPHA)
    for row in range(fill_h):
        alpha = int(120 + 80 * row / max(fill_h, 1))
        pygame.draw.line(ws, (40, 140, 255, alpha), (0, row), (bw-6, row))
    surf.blit(ws, (bx+3, fill_y))
    wave_y = fill_y
    for wx in range(bx+3, bx+bw-3, 8):
        off = int(3 * math.sin(time.time()*4 + wx*0.3))
        pygame.draw.circle(surf, (100,180,255), (wx, wave_y+off), 2)

def draw_bottle_inverted(surf, phase, is_washing):
    pygame.draw.polygon(surf, (70,70,85), bpts)
    draw_wash_level(surf, min(phase * 3.0, 1.0), neck_x, neck_top, neck_w, neck_h)
    if phase > 0.33:
        draw_wash_level(surf, min((phase - 0.33) * 1.5, 1.0),
                        bottle_x, body_top, bottle_w, body_h)
    pygame.draw.lines(surf, (200,200,200), False, bpts, 3)
    pygame.draw.line(surf, (100,180,255), (bottle_x+8, body_top+10),  (bottle_x+8, body_bottom-10), 2)
    pygame.draw.line(surf, (100,180,255), (neck_x+5,   neck_top+5),   (neck_x+5,   neck_bottom-5),  2)

# ─── Arranque ─────────────────────────────────────────────────────────────────
publish_json(PAYLOAD_ZZZ)

running = True

try:
    while running:
        now = time.time()

        # ── Eventos de teclado ────────────────────────────────────────────────
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

        # ── GPIO ──────────────────────────────────────────────────────────────
        process_gpio_buttons(now)

        # ── Auto modo ─────────────────────────────────────────────────────────
        if auto_mode and (now - last_auto_time) >= auto_interval:
            last_auto_time  = now
            virtual_plc_bit = True

        # ── Leer snapshot del hilo Snap7 (no bloquea) ─────────────────────────
        snapshot = get_plc_snapshot()
        plc_trigger = snapshot["trigger"]


        # ── Procesar flancos de estado (solo lógica, sin red) ─────────────────
        process_plc_status_edges(snapshot)

        # ── Trigger de ciclo ──────────────────────────────────────────────────
        effective_trigger = plc_trigger or virtual_plc_bit

        if (not prev_trigger) and effective_trigger and (now - last_trigger_time) > DEBOUNCE_S:
            last_trigger_time = now
            if not cycle_active:
                trigger_cycle()

        prev_trigger = effective_trigger

        if virtual_plc_bit and (now - last_trigger_time) > 0.05:
            virtual_plc_bit = False

        # ── Fin de ciclo ──────────────────────────────────────────────────────
        if cycle_active and now >= cycle_end_time:
            stop_cycle()

        # ── Animación de presión ──────────────────────────────────────────────
        target_p = 3.2 if washing_active else 0.0
        diff = target_p - current_pressure
        if abs(diff) > 0.02:
            current_pressure += animation_speed * (1 if diff > 0 else -1)
            current_pressure  = max(0.0, current_pressure)

        if washing_active:
            elapsed    = max(0.0, now - (cycle_end_time - CYCLE_DURATION_S))
            wash_phase = min(elapsed / CYCLE_DURATION_S, 1.0)

            if current_pressure > 0.5 and random.random() < 0.4:
                spray_jets.append(SprayJet(bottle_cx, neck_bottom + 5))
            if current_pressure > 0.5 and random.random() < 0.5 * (current_pressure / 4.0):
                water_drops.append(WaterDrop(neck_x, neck_bottom, neck_w))

        for d in water_drops[:]:
            d.update()
            if not d.alive or d.y > HEIGHT - 30:
                water_drops.remove(d)
        for j in spray_jets[:]:
            j.update()
            if not j.alive or j.y > neck_bottom:
                spray_jets.remove(j)

        live_execute = execute_total_s + (now - execute_start_time if execute_start_time else 0)
        live_held    = held_total_s    + (now - held_start_time    if held_start_time    else 0)

        # ── Dibujar ───────────────────────────────────────────────────────────
        screen.fill((30, 30, 40))

        sb = pygame.Surface((BOTTLE_W_AREA - 10, 80), pygame.SRCALPHA)
        sb.fill((40, 40, 50))
        pygame.draw.rect(sb, (100, 100, 120), sb.get_rect(), 2)
        screen.blit(sb, (10, 10))

        screen.blit(small_font.render(
            f"Presión: {pressure_value if pressure_value is not None else '--'} bar",
            True, (255, 255, 255)), (20, 25))
        screen.blit(small_font.render(
            f"Status: {status} | Ciclos: {activation_count}",
            True, (255, 255, 255)), (20, 50))

        MAX_P = 5.0
        for i, p in enumerate([5, 4, 3, 2, 1, 0]):
            my = scale_start_y + i * scale_height // 5
            pygame.draw.line(screen, (150,150,150), (scale_x-10, my), (scale_x, my), 2)
            screen.blit(small_font.render(str(p), True, (150,150,150)), (scale_x-25, my-8))
        pygame.draw.rect(screen, (60,60,70), (scale_x-5, scale_start_y, 10, scale_height), 2)
        rt = pygame.transform.rotate(small_font.render("Presión (bar)", True, (150,150,150)), 90)
        screen.blit(rt, (scale_x-55, scale_start_y + scale_height//2 - 40))

        if current_pressure > 0:
            bh2 = int((current_pressure / MAX_P) * scale_height)
            bc  = (0,200,0) if 2.0 <= current_pressure <= 4.5 else (255,100,100)
            by2 = scale_start_y + scale_height - bh2
            pygame.draw.rect(screen, bc, (scale_x-3, by2, 6, bh2))
            pygame.draw.circle(screen, bc, (scale_x, by2-10), 8)
            screen.blit(small_font.render(f"{current_pressure:.1f}", True, (255,255,255)),
                        (scale_x+15, by2-18))

        draw_bottle_inverted(screen, wash_phase if washing_active else 0.0, washing_active)

        nozzle_y = neck_bottom + 10
        draw_nozzle(screen, bottle_cx, nozzle_y)

        for j in spray_jets:
            alpha = int(200 * j.life)
            js = pygame.Surface((j.r*2, j.r*2), pygame.SRCALPHA)
            pygame.draw.circle(js, (80, 160, 255, alpha), (j.r, j.r), j.r)
            screen.blit(js, (int(j.x)-j.r, int(j.y)-j.r))

        for d in water_drops:
            alpha = int(220 * d.life)
            ds = pygame.Surface((d.r*2, d.r*2), pygame.SRCALPHA)
            pygame.draw.circle(ds, (60, 140, 255, alpha), (d.r, d.r), d.r)
            screen.blit(ds, (int(d.x)-d.r, int(d.y)-d.r))

        drain_y = HEIGHT - 58
        pygame.draw.ellipse(screen, (30,80,140),  (bottle_cx-50, drain_y, 100, 14))
        pygame.draw.ellipse(screen, (50,100,180), (bottle_cx-50, drain_y, 100, 14), 2)
        screen.blit(label_font.render("DRENAJE", True, (80,120,180)), (bottle_cx-24, drain_y+1))

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
        screen.blit(label_font.render("BOTELLAS LIMPIAS", True, (0,200,80)), (gr.x+8, gr.y+5))
        gv = big_font.render(str(good_count), True, (0,255,100))
        screen.blit(gv, gv.get_rect(centerx=gr.centerx, y=gr.y+26))

        py += row_h + 6
        br = pygame.Rect(px, py, pw, row_h)
        draw_rounded_rect(screen, (60,20,20), br, radius=8, border=1, border_color=(200,60,60))
        screen.blit(label_font.render("LAVADO DEFICIENTE", True, (220,80,80)), (br.x+8, br.y+5))
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
            "execute":  "Lavado activo\n(run=1)",
            "held":     "Pausa por GPIO\n(feed=1)",
            "complete": "Completado por GPIO\n(done=1)",
            "stopped":  "Abortado/Suspendido\n(stopped=1)",
            "idle":     "Esperando\ntrigger",
        }
        for line in desc.get(machine_state, "").split("\n"):
            dl = label_font.render(line, True, (180,180,200))
            screen.blit(dl, dl.get_rect(centerx=px+pw//2, y=py))
            py += 16

        if washing_active:
            bar_rect = pygame.Rect(px, py+20, pw, 12)
            pygame.draw.rect(screen, (50,50,70), bar_rect, border_radius=6)
            fill_w = int(pw * wash_phase)
            if fill_w > 0:
                pygame.draw.rect(screen, (70,70,85),
                                 pygame.Rect(px, py+20, fill_w, 12), border_radius=6)
            pygame.draw.rect(screen, (80,80,120), bar_rect, 1, border_radius=6)
            pct_txt = label_font.render(f"Lavado: {int(wash_phase*100)}%", True, (140,180,255))
            screen.blit(pct_txt, pct_txt.get_rect(centerx=px+pw//2, y=py-2))

        pygame.display.flip()
        clock.tick(30)

finally:
    _snap7_running = False
    publish_json(PAYLOAD_OFF)
    try:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    except Exception:
        pass
    try:
        GPIO.cleanup()
    except Exception:
        pass
    pygame.quit()
