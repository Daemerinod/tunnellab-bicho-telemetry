"""Verifica puente PLC, actuadores en emergencia, ruido, sensor inalambrico y 10 Hz."""
import importlib.util
import json
import sys
import threading
import time
import urllib.error
import urllib.request

spec = importlib.util.spec_from_file_location("sim_mod", "simulador_bicho.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

FAILS = []


def check(label, cond, detail=""):
    mark = "OK  " if cond else "FAIL"
    if not cond:
        FAILS.append(label)
    print(f"  [{mark}] {label}" + (f" -> {detail}" if detail else ""))


print("=== 1. Offline: PLC, actuadores 0, sensores vivos, ruido, interlock ===")
sim = mod.BichoSimulator(start_state=mod.State.IDLE)
sim.execute("set_state", {"state": "READY"})
sim.execute("inflate_bladder", {"target_bar": 1.85})
for _ in range(80):
    sim.tick(0.1)
sim.execute("start_cutterhead", {"rpm": 3.6})
for _ in range(40):
    sim.tick(0.1)
sim.execute("set_state", {"state": "MINING"})
for _ in range(30):
    sim.tick(0.1)
t_mine = sim.telemetry()
check("excavando", t_mine["mining"] is True and t_mine["extra"]["cutterhead_rpm"] > 1.0)
check("sensor bateria en extra", "bladder_sensor_battery_percent" in t_mine["extra"])
check("sensor senal en extra", "bladder_sensor_signal_ms" in t_mine["extra"])
check("bateria ~100%", t_mine["extra"]["bladder_sensor_battery_percent"] > 90)
press_before = t_mine["extra"]["bladder_pressure_bar"]
pose_before = (t_mine["easting"], t_mine["northing"], t_mine["elevation"])

payload = sim.trigger_plc_estop(source="PLC-TEST")
check("PLC confirma E-STOPPED", payload["ok"] and payload["state"] == "E-STOPPED")
t_estop = sim.telemetry()
check("rpm actuador = 0", t_estop["extra"]["cutterhead_rpm"] == 0.0, str(t_estop["extra"]["cutterhead_rpm"]))
check("propulsion = 0", t_estop["extra"]["propulsion_speed_mm_min"] == 0.0)
check("mining=false", t_estop["mining"] is False)
pressures = []
dofs = []
for _ in range(25):
    sim.tick(0.1)
    snap = sim.telemetry()
    check("actuadores siguen en 0", snap["extra"]["cutterhead_rpm"] == 0.0 and snap["extra"]["propulsion_speed_mm_min"] == 0.0)
    pressures.append(snap["extra"]["bladder_pressure_bar"])
    pressures.append(snap["extra"]["hydroshield_pressure_bar"])
    dofs.append((snap["easting"], snap["northing"], snap["elevation"], snap["roll"], snap["pitch"], snap["heading"]))
check("presion vejiga sigue viva", max(pressures) > 0.2, str(max(pressures)))
check("hydroshield no se apaga en E-STOP", any(p > 0.3 for p in pressures[1::2]))
check("6DOF sigue publicandose", all(len(d) == 6 for d in dofs))
check("ruido blanco en presion (variacion)", len(set(round(p, 3) for p in pressures[0::2])) > 1)
check("ruido blanco en 6DOF", len(set(round(d[3], 3) for d in dofs)) > 1)
check("tbc keys intactas", all(k in t_estop for k in (
    "team", "timestamp", "mining", "chainage", "easting", "northing",
    "elevation", "roll", "pitch", "heading", "extra")))

sim2 = mod.BichoSimulator(start_state=mod.State.IDLE)
sim2.execute("set_state", {"state": "READY"})
sim2.execute("start_cutterhead", {"rpm": 3.0})
for _ in range(20):
    sim2.tick(0.1)
sim2.execute("simulate_fault", {"fault": "bladder_sensor_signal"})
for _ in range(5):
    sim2.tick(0.1)
t_fault = sim2.telemetry()
check("perdida de senal -> FAULT", t_fault["extra"]["state"] == "FAULT", t_fault["extra"]["state"])
check("interlock bladder_sensor", "bladder_sensor" in t_fault["extra"]["safety"]["latched_interlocks"])
check("actuadores 0 en FAULT", t_fault["extra"]["cutterhead_rpm"] == 0.0)
check("sensores siguen en FAULT", t_fault["extra"]["bladder_sensor_signal_ms"] >= 250)

sim3 = mod.BichoSimulator(start_state=mod.State.IDLE)
sim3.execute("set_state", {"state": "READY"})
for _ in range(10):
    sim3.tick(0.1)
sim3.execute("set_bladder_sensor", {"battery_percent": 9.0})
for _ in range(3):
    sim3.tick(0.1)
t_bat = sim3.telemetry()
check("bateria critica -> FAULT", t_bat["extra"]["state"] == "FAULT", t_bat["extra"]["state"])

print("\n=== 2. HTTP: /api/plc/parada, /tbc_endpoint, 10 Hz ===")
handler = mod.BichoRequestHandler
sim_http = mod.BichoSimulator(start_state=mod.State.IDLE)
handler.simulator = sim_http
stop = threading.Event()
mod.SimulationLoop(sim_http, stop).start()
server = mod.ThreadingHTTPServer(("127.0.0.1", 8130), handler)
thread = threading.Thread(target=server.serve_forever, daemon=True)
thread.start()
time.sleep(0.3)
base = "http://127.0.0.1:8130"


def get(path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.status, json.loads(r.read().decode())


def post(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    headers = {"Content-Type": "application/json"} if body is not None else {}
    req = urllib.request.Request(base + path, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode())


code, tel = get("/telemetry")
check("GET /telemetry 200", code == 200)
code, r = post("/command", {"command": "set_state", "params": {"state": "READY"}})
check("READY via /command", code == 200)
code, r = post("/api/plc/parada")
check("POST /api/plc/parada 200", code == 200, str(code))
check("JSON confirmacion", r.get("ok") is True and r.get("state") == "E-STOPPED")
code, tel = get("/telemetry")
check("tras PLC: E-STOPPED", tel["extra"]["state"] == "E-STOPPED")
check("tras PLC: rpm 0", tel["extra"]["cutterhead_rpm"] == 0)
idxs = []
for _ in range(12):
    _, tel = get("/telemetry")
    idxs.append(tel["extra"]["sample_index"])
    time.sleep(0.12)
unique = len(set(idxs))
check("samples distintos a ~10 Hz", unique >= 8, f"{unique} indices {idxs[0]}..{idxs[-1]}")
code, r = post("/tbc_endpoint", tel)
check("POST /tbc_endpoint intacto", code == 200 and r.get("status") == "success")

stop.set()
server.shutdown()
server.server_close()

print("\n" + "=" * 50)
if FAILS:
    print(f"FALLOS: {FAILS}")
    sys.exit(1)
print("Todas las verificaciones pasaron.")
