#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=========================================================================
 B.I.C.H.O. - Backend Simulado / Gemelo Digital   |   UDD Tunnel Lab
=========================================================================
Gemelo digital de la tuneladora a escala funcional del equipo UDD Tunnel
Lab para la Not-a-Boring Competition (The Boring Company).

Implementa:
  * Maquina de estados segura: POWER OFF / E-STOPPED / IDLE / READY /
    MINING / FAULT, con transiciones validadas (nunca se pasa de un
    estado inseguro directamente a excavar).
  * Reinicio seguro (Safe Restart): salir de E-STOPPED, POWER OFF o
    FAULT exige SIEMPRE 2 acciones independientes.
  * Interlocks automaticos: sobrecarga del cabezal de corte, baja
    presion de vejiga, sobrepresion de vejiga y perdida de red.
  * Navegacion 6DOF (easting, northing, elevation, roll, pitch,
    heading) integrada matematicamente solo mientras se excava.
  * API REST local:  GET /telemetry   POST /command   POST /tbc_endpoint
  * Uplink opcional de telemetria por HTTP POST (>= 0.1 Hz).

Solo libreria estandar de Python 3.

Uso:
    python simulador_bicho.py
    python simulador_bicho.py --port 8000 --start-state IDLE
    python simulador_bicho.py --uplink-url http://localhost:8000/tbc_endpoint
"""

import argparse
import json
import math
import random
import sys
import threading
import time
import urllib.error
import urllib.request
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# =========================================================================
# 1. CONFIGURACION Y CONSTANTES FISICAS
# =========================================================================

TEAM_NAME = "UDD Tunnel Lab"
VERSION = "2.0"

TICK_HZ = 10.0                      # frecuencia del lazo de simulacion fisica
DEFAULT_PORT = 8000

# --- Cabezal de corte (cutterhead) ---
CUTTERHEAD_RPM_MAX = 8.0
CUTTERHEAD_RPM_DEFAULT = 3.5
CUTTERHEAD_RPM_MIN_MINING = 1.0     # rpm minimas para autorizar MINING
CUTTERHEAD_RPM_RAMP = 1.2           # rpm/s acelerando
CUTTERHEAD_RPM_BRAKE = 3.0          # rpm/s frenando
CUTTERHEAD_LOAD_MAX_PCT = 100.0     # umbral de sobrecarga
CUTTERHEAD_CURRENT_MAX_A = 32.0     # umbral de sobrecorriente
CUTTERHEAD_AMPS_PER_PCT = 0.32
CUTTERHEAD_TORQUE_MAX_NM = 900.0
OVERLOAD_DEBOUNCE_S = 1.5           # antirebote del interlock de sobrecarga

# --- Vejiga de propulsion (bladder) ---
BLADDER_P_MIN = 1.20                # bar - por debajo se inhibe la propulsion
BLADDER_P_MAX = 3.00                # bar - por encima: alivio + inhibicion
BLADDER_P_TARGET_DEFAULT = 1.80
BLADDER_P_TARGET_MAX = 2.60
BLADDER_INFLATE_RATE = 0.40         # bar/s de inflado
BLADDER_LEAK_RATE = 0.015           # bar/s de fuga normal
BLADDER_LEAK_FAULT_RATE = 0.55      # bar/s de fuga simulada (falla inyectada)
BLADDER_PUMP_STUCK_RATE = 0.50      # bar/s con bomba trabada (falla inyectada)
BLADDER_RELIEF_RATE = 0.90          # bar/s de la valvula de alivio
BLADDER_RELIEF_HYST = 0.25          # bar de histeresis para cerrar el alivio
BLADDER_GRIP_GRACE_S = 6.0          # margen de inflado antes de armar el interlock de baja presion

# --- Propulsion tipo oruga ---
PROPULSION_SPEED_MAX = 60.0         # mm/min
PROPULSION_SPEED_DEFAULT = 42.0     # mm/min
PROPULSION_RAMP = 12.0              # mm/min por segundo

# --- Motores de direccion A (guiñada) y B (cabeceo) ---
STEER_STROKE_MM = 25.0              # recorrido +/- respecto al centro
STEER_RATE_MM_S = 2.0
STEER_YAW_DEG_PER_M = 0.060         # grados de guiñada por mm de A y metro avanzado
STEER_PITCH_DEG_PER_M = 0.040       # grados de cabeceo por mm de B y metro avanzado
PITCH_LIMIT_DEG = 8.0

# --- Hydroshield (presion de soporte del frente) ---
HYDROSHIELD_IDLE_BAR = 1.90
HYDROSHIELD_MINING_BAR = 2.45

# --- Comunicaciones ---
COMMS_TIMEOUT_S = 15.0              # > 10 s (0.1 Hz) para no dar falsos positivos
UPLINK_MAX_FAILURES = 3

# --- Termico ---
AMBIENT_TEMP_C = 24.0

# --- Punto de partida georreferenciado (UTM) ---
START_CHAINAGE_M = 12.450
START_EASTING_M = 342118.640
START_NORTHING_M = 6296004.210
START_ELEVATION_M = -42.180
START_ROLL_DEG = 0.620
START_PITCH_DEG = -1.180
START_HEADING_DEG = 118.440

# --- Subsistemas inhabilitables por interlock ---
SUB_CUTTERHEAD = "cutterhead"
SUB_PROPULSION = "propulsion"
SUB_INFLATION = "inflation"
SUB_STEERING = "steering"


# =========================================================================
# 2. UTILIDADES Y LOGGING
# =========================================================================

def clamp(value, low, high):
    """Acota un valor al rango [low, high]."""
    return low if value < low else (high if value > high else value)


def _stamp():
    return time.strftime("%H:%M:%S", time.localtime())


def _log(icon, tag, message):
    print(f"[{_stamp()}] {icon} {tag:<10}| {message}", flush=True)


def log_info(message):
    _log("ℹ️ ", "INFO", message)


def log_warn(message):
    _log("⚠️ ", "AVISO", message)


def log_command(message):
    _log("📥", "COMANDO", message)


def log_state(message):
    _log("🔄", "ESTADO", message)


def log_interlock(message):
    _log("🚨", "INTERLOCK", message)


def log_net(message):
    _log("📡", "RED", message)


def configure_console():
    """Evita UnicodeEncodeError al redirigir la salida en Windows."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


# =========================================================================
# 3. MAQUINA DE ESTADOS: DEFINICION
# =========================================================================

class State(Enum):
    """Estados permitidos de la tuneladora."""

    POWER_OFF = "POWER OFF"
    E_STOPPED = "E-STOPPED"
    IDLE = "IDLE"
    READY = "READY"
    MINING = "MINING"
    FAULT = "FAULT"


# Grafo de transiciones permitidas. Notese que ni POWER OFF, ni E-STOPPED,
# ni FAULT tienen un camino directo a MINING: para excavar hay que pasar
# obligatoriamente por IDLE y luego por READY.
ALLOWED_TRANSITIONS = {
    State.POWER_OFF: {State.IDLE, State.E_STOPPED},
    State.E_STOPPED: {State.IDLE, State.POWER_OFF},
    State.IDLE: {State.READY, State.E_STOPPED, State.POWER_OFF, State.FAULT},
    State.READY: {State.IDLE, State.MINING, State.E_STOPPED, State.POWER_OFF, State.FAULT},
    State.MINING: {State.READY, State.IDLE, State.E_STOPPED, State.POWER_OFF, State.FAULT},
    State.FAULT: {State.IDLE, State.E_STOPPED, State.POWER_OFF},
}

# Condicion de reinicio seguro: volver a un estado operativo desde una
# parada de emergencia, una perdida de energia o una falla exige DOS
# acciones independientes y en orden.
RESTART_SEQUENCES = {
    State.POWER_OFF: ("power_on", "arm"),
    State.E_STOPPED: ("reset_estop", "arm"),
    State.FAULT: ("clear_fault", "arm"),
}

STATE_ALIASES = {
    "POWER OFF": State.POWER_OFF,
    "POWEROFF": State.POWER_OFF,
    "OFF": State.POWER_OFF,
    "E STOPPED": State.E_STOPPED,
    "ESTOPPED": State.E_STOPPED,
    "E STOP": State.E_STOPPED,
    "ESTOP": State.E_STOPPED,
    "EMERGENCY": State.E_STOPPED,
    "IDLE": State.IDLE,
    "READY": State.READY,
    "MINING": State.MINING,
    "MINE": State.MINING,
    "FAULT": State.FAULT,
}


def parse_state(raw):
    """Convierte texto libre ('ready', 'e_stop', 'POWER OFF') en un State."""
    key = " ".join(str(raw).strip().upper().replace("-", " ").replace("_", " ").split())
    return STATE_ALIASES.get(key)


class Interlock:
    """Enclavamiento de seguridad automatico.

    `active`  -> la condicion fisica esta presente en este instante.
    `latched` -> el enclavamiento quedo enganchado y exige reset manual.
    """

    def __init__(self, key, description, inhibits, forces_fault):
        self.key = key
        self.description = description
        self.inhibits = frozenset(inhibits)
        self.forces_fault = forces_fault
        self.active = False
        self.latched = False
        self.triggered_at = None
        self.trip_count = 0

    def trip(self):
        """Engancha el enclavamiento. Devuelve True solo la primera vez."""
        self.active = True
        if self.latched:
            return False
        self.latched = True
        self.triggered_at = time.time()
        self.trip_count += 1
        return True

    def clear(self):
        self.active = False
        self.latched = False
        self.triggered_at = None

    def snapshot(self):
        return {
            "description": self.description,
            "active": self.active,
            "latched": self.latched,
            "inhibits": sorted(self.inhibits),
            "forces_fault": self.forces_fault,
            "trip_count": self.trip_count,
            "triggered_at": None if self.triggered_at is None else int(self.triggered_at),
        }


# =========================================================================
# 4. GEMELO DIGITAL: ESTADO, FISICA Y SEGURIDAD
# =========================================================================

class BichoSimulator:
    """Gemelo digital del B.I.C.H.O.

    Encapsula la maquina de estados, el modelo fisico simplificado de los
    subsistemas y los enclavamientos de seguridad. Todos los accesos
    publicos son seguros entre hilos.
    """

    def __init__(self, comms_timeout=COMMS_TIMEOUT_S, start_state=State.POWER_OFF):
        self._lock = threading.RLock()
        self.comms_timeout = float(comms_timeout)

        # --- Maquina de estados ---
        self.state = State.POWER_OFF
        self.previous_state = State.POWER_OFF
        self.state_since = time.time()
        self.estop_engaged = False
        self._restart_done = set()

        # --- Navegacion 6DOF ---
        self.chainage = START_CHAINAGE_M
        self.easting = START_EASTING_M
        self.northing = START_NORTHING_M
        self.elevation = START_ELEVATION_M
        self.roll = START_ROLL_DEG
        self.pitch = START_PITCH_DEG
        self.heading = START_HEADING_DEG
        self.yaw_rate_deg_s = 0.0
        self.vib_roll = 0.0
        self.vib_pitch = 0.0
        self._vib_phase = 0.0

        # --- Cabezal de corte ---
        self.cutterhead_running = False
        self.cutterhead_rpm = 0.0
        self.cutterhead_rpm_target = 0.0
        self.cutterhead_load_pct = 0.0
        self.cutterhead_current_a = 0.0
        self.cutterhead_torque_nm = 0.0
        self.cutterhead_temp_c = AMBIENT_TEMP_C

        # --- Vejiga y propulsion ---
        self.bladder_pressure = 0.0
        self.bladder_target = 0.0
        self.bladder_inflating = False
        self.relief_valve_open = False
        self.propulsion_enabled = False
        self.propulsion_speed = 0.0
        self.propulsion_speed_target = PROPULSION_SPEED_DEFAULT
        self.track_left_mm_min = 0.0
        self.track_right_mm_min = 0.0
        self.thrust_force_kn = 0.0
        self.propulsion_temp_c = AMBIENT_TEMP_C

        # --- Direccion ---
        self.steer_a_mm = 0.0
        self.steer_b_mm = 0.0
        self.steer_a_target_mm = 0.0
        self.steer_b_target_mm = 0.0

        # --- Hydroshield y terreno ---
        self.hydroshield_pressure = 0.0
        self.ground_hardness = 1.0
        self._ground_phase = 0.0
        self._ground_walk = 0.0

        # --- Enclavamientos ---
        self.interlocks = {
            "cutterhead_overload": Interlock(
                "cutterhead_overload",
                "Sobrecorriente o sobrecarga en el cabezal de corte",
                {SUB_CUTTERHEAD, SUB_PROPULSION},
                forces_fault=True,
            ),
            "bladder_underpressure": Interlock(
                "bladder_underpressure",
                "Presion de la vejiga de propulsion por debajo de P_min",
                {SUB_PROPULSION},
                forces_fault=False,
            ),
            "bladder_overpressure": Interlock(
                "bladder_overpressure",
                "Sobrepresion en la vejiga de propulsion (P > P_max)",
                {SUB_PROPULSION, SUB_INFLATION},
                forces_fault=True,
            ),
            "comms_loss": Interlock(
                "comms_loss",
                "Perdida de comunicacion de red: aborto global",
                {SUB_CUTTERHEAD, SUB_PROPULSION, SUB_INFLATION, SUB_STEERING},
                forces_fault=True,
            ),
        }

        # --- Fallas inyectadas desde POST /command ---
        self.forced_overload = False
        self.bladder_leak = False
        self.pump_stuck = False
        self.forced_comms_loss = False

        # --- Contadores y watchdog ---
        self.boot_ts = time.time()
        self.last_contact_ts = time.time()
        self.last_command = None
        self.commands_received = 0
        self.mining_seconds = 0.0
        self.uplink_ok = None
        self.uplink_failures = 0
        self._overload_timer = 0.0
        self._grip_timer = 0.0

        self._commands = {
            "power_on": self._cmd_power_on,
            "power_off": self._cmd_power_off,
            "estop": self._cmd_estop,
            "reset_estop": self._cmd_reset_estop,
            "arm": self._cmd_arm,
            "clear_fault": self._cmd_clear_fault,
            "set_state": self._cmd_set_state,
            "start_cutterhead": self._cmd_start_cutterhead,
            "stop_cutterhead": self._cmd_stop_cutterhead,
            "set_cutterhead_rpm": self._cmd_set_cutterhead_rpm,
            "steer": self._cmd_steer,
            "center_steering": self._cmd_center_steering,
            "set_propulsion_speed": self._cmd_set_propulsion_speed,
            "inflate_bladder": self._cmd_inflate_bladder,
            "deflate_bladder": self._cmd_deflate_bladder,
            "simulate_fault": self._cmd_simulate_fault,
            "heartbeat": self._cmd_heartbeat,
        }

        if start_state is not State.POWER_OFF:
            self.state = start_state
            self.previous_state = State.POWER_OFF
            self._on_enter_state(start_state, State.POWER_OFF)
            log_warn(
                f"Estado inicial forzado a {start_state.value} por linea de comandos "
                "(modo desarrollo, se omite la secuencia de encendido)."
            )

    # -------------------------------------------------------------------
    # 4.1 Transiciones de estado
    # -------------------------------------------------------------------

    def _set_state(self, new_state, reason):
        if new_state is self.state:
            return
        previous = self.state
        self.previous_state = previous
        self.state = new_state
        self.state_since = time.time()
        self._restart_done = set()
        log_state(f"{previous.value}  ->  {new_state.value}   ({reason})")
        self._on_enter_state(new_state, previous)

    def _on_enter_state(self, new_state, previous):
        """Acciones automaticas al entrar en un estado."""
        if new_state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            self._stop_all_actuation()
            if new_state is State.E_STOPPED:
                self.estop_engaged = True
            return

        if new_state is State.IDLE:
            # IDLE: energizado pero sin actuacion. Nada gira, nada avanza.
            self._stop_subsystem(SUB_CUTTERHEAD)
            self._stop_subsystem(SUB_PROPULSION)
            self._stop_subsystem(SUB_INFLATION)
            self._grip_timer = 0.0
            return

        if new_state is State.READY:
            # READY: se presuriza la vejiga para poder agarrar el tunel.
            self.bladder_inflating = True
            self.bladder_target = max(self.bladder_target, BLADDER_P_TARGET_DEFAULT)
            self.propulsion_speed_target = self.propulsion_speed_target or PROPULSION_SPEED_DEFAULT
            return

        if new_state is State.MINING:
            self.bladder_inflating = True
            self.bladder_target = max(self.bladder_target, BLADDER_P_TARGET_DEFAULT)
            if self.propulsion_speed_target <= 0.0:
                self.propulsion_speed_target = PROPULSION_SPEED_DEFAULT

    def _stop_subsystem(self, subsystem):
        if subsystem == SUB_CUTTERHEAD:
            self.cutterhead_running = False
            self.cutterhead_rpm_target = 0.0
        elif subsystem == SUB_PROPULSION:
            self.propulsion_enabled = False
            self.propulsion_speed_target = 0.0
        elif subsystem == SUB_INFLATION:
            self.bladder_inflating = False
            self.bladder_target = 0.0
        elif subsystem == SUB_STEERING:
            self.steer_a_target_mm = self.steer_a_mm
            self.steer_b_target_mm = self.steer_b_mm

    def _stop_all_actuation(self):
        for subsystem in (SUB_CUTTERHEAD, SUB_PROPULSION, SUB_INFLATION, SUB_STEERING):
            self._stop_subsystem(subsystem)

    def _inhibited(self, subsystem):
        """True si el subsistema esta inhabilitado por estado o interlock."""
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return True
        if self.estop_engaged:
            return True
        return any(
            lock.latched and subsystem in lock.inhibits
            for lock in self.interlocks.values()
        )

    def _latched_keys(self):
        return sorted(key for key, lock in self.interlocks.items() if lock.latched)

    # -------------------------------------------------------------------
    # 4.2 Lazo de simulacion fisica
    # -------------------------------------------------------------------

    def tick(self, dt):
        """Avanza la simulacion `dt` segundos y evalua los interlocks."""
        with self._lock:
            now = time.time()
            self._update_ground(dt)
            self._update_cutterhead(dt)
            self._update_bladder(dt)
            self._update_propulsion(dt)
            self._update_steering(dt)
            self._update_hydroshield(dt)
            self._update_navigation(dt)
            self._update_thermal(dt)
            if self.state is State.MINING:
                self.mining_seconds += dt
            self._check_interlocks(now, dt)

    def _update_ground(self, dt):
        """Dureza del terreno: deriva lenta + caminata aleatoria acotada."""
        self._ground_phase += dt * 0.05
        self._ground_walk = clamp(
            self._ground_walk + random.uniform(-0.05, 0.05) * dt, -0.08, 0.08
        )
        self.ground_hardness = clamp(
            1.0 + 0.10 * math.sin(self._ground_phase) + self._ground_walk, 0.82, 1.18
        )

    def _update_cutterhead(self, dt):
        allowed = self.cutterhead_running and not self._inhibited(SUB_CUTTERHEAD)
        target = self.cutterhead_rpm_target if allowed else 0.0

        if self.cutterhead_rpm < target:
            self.cutterhead_rpm = min(target, self.cutterhead_rpm + CUTTERHEAD_RPM_RAMP * dt)
        else:
            self.cutterhead_rpm = max(target, self.cutterhead_rpm - CUTTERHEAD_RPM_BRAKE * dt)

        if self.cutterhead_rpm <= 0.01:
            self.cutterhead_rpm = 0.0
            self.cutterhead_load_pct = 0.0
            self.cutterhead_current_a = 0.0
            self.cutterhead_torque_nm = 0.0
            return

        # Carga = f(rpm, dureza del terreno, empuje aplicado)
        load = 8.0 + 95.0 * (self.cutterhead_rpm / CUTTERHEAD_RPM_MAX) * self.ground_hardness
        if self.state is State.MINING and self.propulsion_speed > 1.0:
            load *= 1.0 + 0.55 * (self.propulsion_speed / PROPULSION_SPEED_MAX)
        if self.forced_overload:
            load += 55.0
        load += random.uniform(-1.5, 1.5)

        self.cutterhead_load_pct = max(0.0, load)
        self.cutterhead_current_a = max(
            0.0, self.cutterhead_load_pct * CUTTERHEAD_AMPS_PER_PCT + random.uniform(-0.25, 0.25)
        )
        self.cutterhead_torque_nm = CUTTERHEAD_TORQUE_MAX_NM * self.cutterhead_load_pct / 100.0

    def _update_bladder(self, dt):
        inflate_ok = self.bladder_inflating and not self._inhibited(SUB_INFLATION)
        pressure = self.bladder_pressure

        if self.pump_stuck:
            # Valvula de inflado trabada: sube ignorando la consigna.
            pressure += BLADDER_PUMP_STUCK_RATE * dt
        elif inflate_ok and self.bladder_target > 0.0:
            error = self.bladder_target - pressure
            if error > 0.0:
                pressure += min(BLADDER_INFLATE_RATE, error * 2.5) * dt
            elif error < -0.05:
                pressure -= min(BLADDER_INFLATE_RATE, -error * 2.0) * dt

        pressure -= BLADDER_LEAK_RATE * dt
        if self.bladder_leak:
            pressure -= BLADDER_LEAK_FAULT_RATE * dt
        if self.relief_valve_open:
            pressure -= BLADDER_RELIEF_RATE * dt
        if not inflate_ok and not self.pump_stuck and self.state is State.POWER_OFF:
            pressure -= 0.10 * dt          # sin energia la vejiga se despresuriza

        self.bladder_pressure = max(0.0, pressure)

        # Temporizador de agarre: cuenta el tiempo desde que se ordeno
        # presurizar la vejiga. Da margen a la bomba para alcanzar P_min
        # antes de armar el interlock de baja presion.
        gripping = inflate_ok and self.bladder_target >= BLADDER_P_MIN
        self._grip_timer = self._grip_timer + dt if gripping else 0.0

    def _update_propulsion(self, dt):
        allowed = (
            self.state is State.MINING
            and not self._inhibited(SUB_PROPULSION)
            and self.bladder_pressure >= BLADDER_P_MIN
        )
        self.propulsion_enabled = allowed
        target = self.propulsion_speed_target if allowed else 0.0

        if self.propulsion_speed < target:
            self.propulsion_speed = min(target, self.propulsion_speed + PROPULSION_RAMP * dt)
        else:
            self.propulsion_speed = max(target, self.propulsion_speed - PROPULSION_RAMP * dt)
        if self.propulsion_speed < 0.05:
            self.propulsion_speed = 0.0

        # Orugas de propulsion: el motor A introduce un diferencial de velocidad.
        differential = self.steer_a_mm / STEER_STROKE_MM
        self.track_left_mm_min = self.propulsion_speed * (1.0 + 0.25 * differential)
        self.track_right_mm_min = self.propulsion_speed * (1.0 - 0.25 * differential)
        self.thrust_force_kn = self.bladder_pressure * 9.5 * (
            0.35 + 0.65 * self.propulsion_speed / PROPULSION_SPEED_MAX
        )

    def _update_steering(self, dt):
        if self._inhibited(SUB_STEERING):
            self.steer_a_target_mm = self.steer_a_mm
            self.steer_b_target_mm = self.steer_b_mm
            return
        step = STEER_RATE_MM_S * dt
        for axis in ("a", "b"):
            current = getattr(self, f"steer_{axis}_mm")
            goal = getattr(self, f"steer_{axis}_target_mm")
            delta = clamp(goal - current, -step, step)
            setattr(self, f"steer_{axis}_mm", current + delta)

    def _update_hydroshield(self, dt):
        if self.state in (State.POWER_OFF, State.E_STOPPED):
            target = 0.0
        elif self.state is State.MINING:
            target = HYDROSHIELD_MINING_BAR
        else:
            target = HYDROSHIELD_IDLE_BAR
        self.hydroshield_pressure += (target - self.hydroshield_pressure) * min(1.0, dt / 2.5)
        if target > 0.0:
            self.hydroshield_pressure += 0.02 * math.sin(time.time() * 1.3)
        self.hydroshield_pressure = max(0.0, self.hydroshield_pressure)

    def _update_navigation(self, dt):
        """Integra los 6 grados de libertad. Solo avanza en MINING."""
        # Vibracion mecanica proporcional a las rpm del cabezal.
        self._vib_phase += dt * 2.0 * math.pi * 1.7
        amplitude = 0.35 * (self.cutterhead_rpm / CUTTERHEAD_RPM_MAX)
        self.vib_roll = amplitude * math.sin(self._vib_phase)
        self.vib_pitch = amplitude * 0.5 * math.sin(self._vib_phase * 0.7)

        if self.state is not State.MINING or self.propulsion_speed <= 0.0:
            self.yaw_rate_deg_s = 0.0
            return

        ds = (self.propulsion_speed / 60000.0) * dt      # mm/min -> m en este tick
        if ds <= 0.0:
            return

        d_heading = STEER_YAW_DEG_PER_M * self.steer_a_mm * ds
        self.heading = (self.heading + d_heading) % 360.0
        self.yaw_rate_deg_s = d_heading / dt if dt > 0 else 0.0
        self.pitch = clamp(
            self.pitch + STEER_PITCH_DEG_PER_M * self.steer_b_mm * ds,
            -PITCH_LIMIT_DEG,
            PITCH_LIMIT_DEG,
        )
        self.roll += (0.02 * self.steer_a_mm - self.roll * 0.5) * ds

        heading_rad = math.radians(self.heading)
        pitch_rad = math.radians(self.pitch)
        horizontal = ds * math.cos(pitch_rad)

        self.chainage += ds
        self.easting += horizontal * math.sin(heading_rad)
        self.northing += horizontal * math.cos(heading_rad)
        self.elevation += ds * math.sin(pitch_rad)

    def _update_thermal(self, dt):
        tau = 60.0
        head_target = AMBIENT_TEMP_C + 0.50 * self.cutterhead_load_pct
        prop_target = AMBIENT_TEMP_C + 0.35 * (self.propulsion_speed / PROPULSION_SPEED_MAX) * 100.0
        self.cutterhead_temp_c += (head_target - self.cutterhead_temp_c) * min(1.0, dt / tau)
        self.propulsion_temp_c += (prop_target - self.propulsion_temp_c) * min(1.0, dt / tau)

    # -------------------------------------------------------------------
    # 4.3 Interlocks automaticos (gatillos de falla)
    # -------------------------------------------------------------------

    def _check_interlocks(self, now, dt):
        # --- 1. Sobrecorriente / sobrecarga del cabezal de corte ---
        lock = self.interlocks["cutterhead_overload"]
        overload = self.cutterhead_rpm > 0.0 and (
            self.cutterhead_load_pct > CUTTERHEAD_LOAD_MAX_PCT
            or self.cutterhead_current_a > CUTTERHEAD_CURRENT_MAX_A
        )
        lock.active = overload
        self._overload_timer = (
            self._overload_timer + dt if overload else max(0.0, self._overload_timer - dt)
        )
        if self._overload_timer >= OVERLOAD_DEBOUNCE_S:
            self._trip(
                lock,
                f"carga {self.cutterhead_load_pct:.0f} % / {self.cutterhead_current_a:.1f} A "
                f"(limites {CUTTERHEAD_LOAD_MAX_PCT:.0f} % / {CUTTERHEAD_CURRENT_MAX_A:.1f} A)",
            )

        # --- 2. Baja presion en la vejiga de propulsion ---
        lock = self.interlocks["bladder_underpressure"]
        armed = (
            self.state in (State.READY, State.MINING)
            and self.bladder_target >= BLADDER_P_MIN
            and self._grip_timer >= BLADDER_GRIP_GRACE_S
        )
        lock.active = armed and self.bladder_pressure < BLADDER_P_MIN
        if lock.active:
            self._trip(
                lock,
                f"P_vejiga {self.bladder_pressure:.2f} bar < P_min {BLADDER_P_MIN:.2f} bar",
            )

        # --- 3. Sobrepresion en la vejiga de propulsion ---
        lock = self.interlocks["bladder_overpressure"]
        lock.active = self.bladder_pressure > BLADDER_P_MAX
        if lock.active:
            if not self.relief_valve_open:
                self.relief_valve_open = True
                log_interlock("Valvula de alivio de presion ABIERTA.")
            self._trip(
                lock,
                f"P_vejiga {self.bladder_pressure:.2f} bar > P_max {BLADDER_P_MAX:.2f} bar",
            )
        elif self.relief_valve_open and self.bladder_pressure <= BLADDER_P_MAX - BLADDER_RELIEF_HYST:
            self.relief_valve_open = False
            log_interlock(
                f"Valvula de alivio CERRADA (P_vejiga {self.bladder_pressure:.2f} bar)."
            )

        # --- 4. Perdida de comunicacion de red ---
        lock = self.interlocks["comms_loss"]
        silence = now - self.last_contact_ts
        watchdog_armed = self.state in (State.READY, State.MINING)
        lock.active = self.forced_comms_loss or (watchdog_armed and silence > self.comms_timeout)
        if lock.active:
            detail = (
                "perdida forzada por comando de prueba"
                if self.forced_comms_loss
                else f"sin trafico durante {silence:.1f} s (timeout {self.comms_timeout:.1f} s)"
            )
            self._trip(lock, detail)

    def _trip(self, lock, detail):
        """Engancha un interlock y aplica sus acciones de mitigacion."""
        if not lock.trip():
            return

        log_interlock(f"DISPARADO '{lock.key}': {lock.description}.")
        log_interlock(f"   Causa: {detail}")
        log_interlock(f"   Inhabilita: {', '.join(sorted(lock.inhibits))}")

        for subsystem in lock.inhibits:
            self._stop_subsystem(subsystem)

        if lock.key == "bladder_overpressure":
            self.relief_valve_open = True

        if lock.forces_fault:
            if self.state not in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
                self._set_state(State.FAULT, f"interlock '{lock.key}'")
            else:
                self._stop_all_actuation()
        elif self.state is State.MINING and SUB_PROPULSION in lock.inhibits:
            # Degradacion segura: sin propulsion no se puede excavar.
            self._set_state(State.READY, f"excavacion abortada por interlock '{lock.key}'")

    def _condition_present(self, key):
        """Reevalua en el instante si la causa fisica de un interlock persiste."""
        if key == "cutterhead_overload":
            return self.cutterhead_rpm > 0.0 and (
                self.cutterhead_load_pct > CUTTERHEAD_LOAD_MAX_PCT
                or self.cutterhead_current_a > CUTTERHEAD_CURRENT_MAX_A
            )
        if key == "bladder_underpressure":
            return (
                self.bladder_target >= BLADDER_P_MIN
                and self.bladder_pressure < BLADDER_P_MIN
                and self.state in (State.READY, State.MINING)
            )
        if key == "bladder_overpressure":
            return self.bladder_pressure > BLADDER_P_MAX
        if key == "comms_loss":
            return self.forced_comms_loss or (
                time.time() - self.last_contact_ts > self.comms_timeout
            )
        return False

    # -------------------------------------------------------------------
    # 4.4 Telemetria (formato obligatorio TBC)
    # -------------------------------------------------------------------

    def touch_comms(self):
        """Refresca el watchdog de red: hubo trafico con la superficie."""
        with self._lock:
            self.last_contact_ts = time.time()

    def telemetry(self):
        with self._lock:
            now = time.time()
            return {
                "team": TEAM_NAME,
                "timestamp": int(now),
                "mining": self.state is State.MINING,
                "chainage": round(self.chainage, 3),
                "easting": round(self.easting, 3),
                "northing": round(self.northing, 3),
                "elevation": round(self.elevation, 3),
                "roll": round(self.roll + self.vib_roll, 3),
                "pitch": round(self.pitch + self.vib_pitch, 3),
                "heading": round(self.heading % 360.0, 3),
                "extra": self._extra_block(now),
            }

    def _extra_block(self, now):
        silence = now - self.last_contact_ts
        return {
            # --- Claves planas (compatibilidad con el dashboard actual) ---
            "cutterhead_rpm": round(self.cutterhead_rpm, 2),
            "cutterhead_load_percent": round(self.cutterhead_load_pct, 1),
            "bladder_pressure_bar": round(self.bladder_pressure, 3),
            "hydroshield_pressure_bar": round(self.hydroshield_pressure, 3),
            "propulsion_speed_mm_min": round(self.propulsion_speed, 2),

            # --- Maquina de estados ---
            "state": self.state.value,
            "previous_state": self.previous_state.value,
            "state_elapsed_s": round(now - self.state_since, 1),
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now)) + "Z",

            "cutterhead": {
                "running": self.cutterhead_running,
                "rpm": round(self.cutterhead_rpm, 2),
                "rpm_setpoint": round(self.cutterhead_rpm_target, 2),
                "load_percent": round(self.cutterhead_load_pct, 1),
                "current_a": round(self.cutterhead_current_a, 2),
                "torque_nm": round(self.cutterhead_torque_nm, 1),
                "motor_temp_c": round(self.cutterhead_temp_c, 1),
                "load_limit_percent": CUTTERHEAD_LOAD_MAX_PCT,
                "current_limit_a": CUTTERHEAD_CURRENT_MAX_A,
                "inhibited": self._inhibited(SUB_CUTTERHEAD),
            },
            "propulsion": {
                "enabled": self.propulsion_enabled,
                "speed_mm_min": round(self.propulsion_speed, 2),
                "speed_setpoint_mm_min": round(self.propulsion_speed_target, 2),
                "track_left_mm_min": round(self.track_left_mm_min, 2),
                "track_right_mm_min": round(self.track_right_mm_min, 2),
                "thrust_force_kn": round(self.thrust_force_kn, 2),
                "motor_temp_c": round(self.propulsion_temp_c, 1),
                "inhibited": self._inhibited(SUB_PROPULSION),
            },
            "bladder": {
                "pressure_bar": round(self.bladder_pressure, 3),
                "setpoint_bar": round(self.bladder_target, 3),
                "p_min_bar": BLADDER_P_MIN,
                "p_max_bar": BLADDER_P_MAX,
                "inflating": self.bladder_inflating and not self._inhibited(SUB_INFLATION),
                "relief_valve_open": self.relief_valve_open,
                "gripped": self.bladder_pressure >= BLADDER_P_MIN,
            },
            "steering": {
                "motor_a_mm": round(self.steer_a_mm, 2),
                "motor_b_mm": round(self.steer_b_mm, 2),
                "motor_a_setpoint_mm": round(self.steer_a_target_mm, 2),
                "motor_b_setpoint_mm": round(self.steer_b_target_mm, 2),
                "stroke_limit_mm": STEER_STROKE_MM,
                "yaw_rate_deg_s": round(self.yaw_rate_deg_s, 4),
                "inhibited": self._inhibited(SUB_STEERING),
            },
            "navigation": {
                "advance_rate_mm_min": round(self.propulsion_speed, 2),
                "chainage_m": round(self.chainage, 3),
                "mined_length_m": round(self.chainage - START_CHAINAGE_M, 3),
                "mining_time_s": round(self.mining_seconds, 1),
                "ground_hardness_index": round(self.ground_hardness, 3),
            },
            "safety": {
                "estop_engaged": self.estop_engaged,
                "active_interlocks": sorted(
                    key for key, lock in self.interlocks.items() if lock.active
                ),
                "latched_interlocks": self._latched_keys(),
                "interlocks": {key: lock.snapshot() for key, lock in self.interlocks.items()},
                "safe_restart": self._restart_status(),
                "allowed_transitions": sorted(s.value for s in ALLOWED_TRANSITIONS[self.state]),
            },
            "comms": {
                "ok": not self.interlocks["comms_loss"].latched,
                "seconds_since_last_contact": round(silence, 2),
                "timeout_s": self.comms_timeout,
                "uplink_ok": self.uplink_ok,
                "uplink_failures": self.uplink_failures,
            },
            "sim": {
                "version": VERSION,
                "tick_hz": TICK_HZ,
                "uptime_s": round(now - self.boot_ts, 1),
                "commands_received": self.commands_received,
                "last_command": self.last_command,
                "injected_faults": sorted(
                    name
                    for name, flag in (
                        ("cutterhead_overload", self.forced_overload),
                        ("bladder_underpressure", self.bladder_leak),
                        ("bladder_overpressure", self.pump_stuck),
                        ("comms_loss", self.forced_comms_loss),
                    )
                    if flag
                ),
            },
        }

    def _restart_status(self):
        sequence = RESTART_SEQUENCES.get(self.state)
        if sequence is None:
            return {"required": False}
        return {
            "required": True,
            "independent_actions_required": len(sequence),
            "sequence": list(sequence),
            "completed": [step for step in sequence if step in self._restart_done],
            "pending": [step for step in sequence if step not in self._restart_done],
        }

    def status_summary(self):
        with self._lock:
            return {
                "ok": True,
                "team": TEAM_NAME,
                "state": self.state.value,
                "mining": self.state is State.MINING,
                "estop_engaged": self.estop_engaged,
                "latched_interlocks": self._latched_keys(),
                "safe_restart": self._restart_status(),
                "uptime_s": round(time.time() - self.boot_ts, 1),
            }

    # -------------------------------------------------------------------
    # 4.5 Modo manual / ingenieria: POST /command
    # -------------------------------------------------------------------

    def execute(self, command, params):
        """Ejecuta un comando. Devuelve (codigo_http, payload)."""
        with self._lock:
            self.last_contact_ts = time.time()
            command = str(command).strip().lower()
            params = params if isinstance(params, dict) else {}

            handler = self._commands.get(command)
            if handler is None:
                log_warn(f"Comando desconocido: '{command}'")
                return 400, self._command_response(
                    False,
                    command,
                    f"Comando desconocido '{command}'. Comandos validos: "
                    f"{', '.join(sorted(self._commands))}.",
                )

            self.commands_received += 1
            self.last_command = command
            log_command(f"'{command}' params={json.dumps(params, ensure_ascii=False)} "
                        f"[estado={self.state.value}]")
            try:
                ok, message = handler(params)
            except (TypeError, ValueError) as exc:
                log_warn(f"'{command}' con parametros invalidos: {exc}")
                return 400, self._command_response(
                    False, command, f"Parametros invalidos: {exc}"
                )

            if ok:
                log_info(f"'{command}' ACEPTADO -> {message}")
            else:
                log_warn(f"'{command}' RECHAZADO -> {message}")
            return (200 if ok else 409), self._command_response(ok, command, message)

    def _command_response(self, ok, command, message):
        return {
            "ok": ok,
            "command": command,
            "message": message,
            "state": self.state.value,
            "mining": self.state is State.MINING,
            "estop_engaged": self.estop_engaged,
            "active_interlocks": sorted(
                key for key, lock in self.interlocks.items() if lock.active
            ),
            "latched_interlocks": self._latched_keys(),
            "safe_restart": self._restart_status(),
            "timestamp": int(time.time()),
        }

    # --- Energia, emergencia y reinicio seguro -------------------------

    def _mark_restart_step(self, step, note):
        """Registra un paso de la secuencia de reinicio seguro (2 acciones)."""
        sequence = RESTART_SEQUENCES.get(self.state)
        if sequence is None or step not in sequence:
            return False, (
                f"El paso '{step}' no corresponde al estado {self.state.value}."
            )

        index = sequence.index(step)
        missing = [s for s in sequence[:index] if s not in self._restart_done]
        if missing:
            return False, (
                "Secuencia de reinicio seguro incompleta: primero debe ejecutar "
                f"'{missing[0]}' antes de '{step}'."
            )
        if step in self._restart_done:
            pending = [s for s in sequence if s not in self._restart_done]
            return True, (
                f"El paso '{step}' ya estaba registrado. Falta(n): {', '.join(pending)}."
            )

        is_last = index == len(sequence) - 1
        if is_last:
            blocked = self._restart_blockers()
            if blocked:
                return False, (
                    "Rearme bloqueado: " + "; ".join(blocked)
                )

        self._restart_done.add(step)
        done, total = len(self._restart_done), len(sequence)
        log_info(f"Reinicio seguro {done}/{total}: {note}")

        if done < total:
            pending = [s for s in sequence if s not in self._restart_done]
            return True, (
                f"Accion {done}/{total} registrada ({note}). "
                f"Falta(n) {len(pending)} accion(es) independiente(s): {', '.join(pending)}."
            )

        self._set_state(
            State.IDLE,
            f"reinicio seguro completado con {total} acciones independientes",
        )
        return True, "Reinicio seguro completo. Maquina en IDLE."

    def _restart_blockers(self):
        blockers = []
        if self.estop_engaged:
            blockers.append("el hongo de emergencia sigue engachado (use 'reset_estop')")
        latched = self._latched_keys()
        if latched:
            blockers.append(f"interlocks aun enganchados: {', '.join(latched)}")
        return blockers

    def _cmd_power_on(self, params):
        if self.state is not State.POWER_OFF:
            return False, f"La maquina ya esta energizada (estado {self.state.value})."
        return self._mark_restart_step("power_on", "energia principal aplicada")

    def _cmd_power_off(self, params):
        if self.state is State.POWER_OFF:
            return True, "La maquina ya estaba sin energia."
        self._set_state(State.POWER_OFF, "corte de energia solicitado por el operador")
        return True, (
            "Energia cortada y actuacion detenida. Se requieren 2 acciones "
            "independientes ('power_on' + 'arm') para volver a operar."
        )

    def _cmd_estop(self, params):
        reason = str(params.get("reason", "activacion manual del operador"))
        if self.state is State.E_STOPPED:
            self.estop_engaged = True
            return True, "E-STOP ya estaba activo."
        self._set_state(State.E_STOPPED, f"PARADA DE EMERGENCIA ({reason})")
        return True, (
            "E-STOP activado: cabezal, propulsion, inflado y direccion detenidos. "
            "Se requieren 2 acciones independientes ('reset_estop' + 'arm') para reiniciar."
        )

    def _cmd_reset_estop(self, params):
        if self.state is not State.E_STOPPED:
            if self.estop_engaged:
                self.estop_engaged = False
                return True, "Hongo de emergencia liberado."
            return False, f"No hay E-STOP activo (estado {self.state.value})."
        self.estop_engaged = False
        return self._mark_restart_step("reset_estop", "hongo de emergencia liberado")

    def _cmd_arm(self, params):
        if self.state not in RESTART_SEQUENCES:
            return False, (
                f"'arm' solo aplica en {', '.join(s.value for s in RESTART_SEQUENCES)}. "
                f"Estado actual: {self.state.value}."
            )
        return self._mark_restart_step("arm", "rearme confirmado por el operador")

    def _cmd_clear_fault(self, params):
        # Retira primero las fallas inyectadas por el modo ingenieria.
        self.forced_overload = False
        self.bladder_leak = False
        self.pump_stuck = False
        self.forced_comms_loss = False
        self.last_contact_ts = time.time()
        self._overload_timer = 0.0

        blocking = []
        cleared = []
        for key, lock in self.interlocks.items():
            if not lock.latched:
                continue
            if self._condition_present(key):
                blocking.append(key)
            else:
                lock.clear()
                cleared.append(key)
                log_interlock(f"Interlock '{key}' reseteado por el operador.")

        if blocking:
            return False, (
                "No se puede resetear: la condicion fisica persiste en "
                f"{', '.join(blocking)}. Corrija la causa y reintente."
            )

        if self.state is not State.FAULT:
            return True, (
                f"Interlocks reseteados ({', '.join(cleared) if cleared else 'ninguno pendiente'})."
            )
        return self._mark_restart_step(
            "clear_fault",
            f"interlocks reseteados ({', '.join(cleared) if cleared else 'ninguno'})",
        )

    # --- Cambio de estado general --------------------------------------

    def _cmd_set_state(self, params):
        raw = params.get("state", params.get("target"))
        if raw is None:
            return False, "Falta el parametro 'state' (IDLE, READY, MINING, ...)."
        target = parse_state(raw)
        if target is None:
            return False, (
                f"Estado desconocido '{raw}'. Validos: "
                f"{', '.join(s.value for s in State)}."
            )
        return self._request_state(target, params)

    def _request_state(self, target, params):
        if target is self.state:
            return True, f"La maquina ya se encuentra en {target.value}."

        if target is State.POWER_OFF:
            return self._cmd_power_off(params)
        if target is State.E_STOPPED:
            return self._cmd_estop({"reason": "solicitado por comando set_state"})
        if target is State.FAULT:
            return self._cmd_simulate_fault({"fault": "generic"})

        if target not in ALLOWED_TRANSITIONS[self.state]:
            allowed = ", ".join(sorted(s.value for s in ALLOWED_TRANSITIONS[self.state]))
            return False, (
                f"Transicion insegura BLOQUEADA: {self.state.value} -> {target.value}. "
                f"Transiciones permitidas desde {self.state.value}: {allowed}."
            )

        if self.state in RESTART_SEQUENCES:
            sequence = " + ".join(RESTART_SEQUENCES[self.state])
            return False, (
                f"Salir de {self.state.value} exige la secuencia de reinicio seguro "
                f"de 2 acciones independientes: {sequence}."
            )

        ok, why = self._preconditions(target)
        if not ok:
            return False, why

        self._set_state(target, "solicitado por el operador")
        return True, f"Estado {target.value} activo."

    def _preconditions(self, target):
        latched = self._latched_keys()
        if target in (State.READY, State.MINING):
            if self.estop_engaged:
                return False, "El hongo de emergencia sigue engachado."
            if latched:
                return False, (
                    f"Interlocks enganchados: {', '.join(latched)}. "
                    "Ejecute 'clear_fault' antes de habilitar la maquina."
                )
        if target is State.MINING:
            if not self.cutterhead_running or self.cutterhead_rpm_target < CUTTERHEAD_RPM_MIN_MINING:
                return False, (
                    "Para excavar el cabezal de corte debe estar en marcha con al menos "
                    f"{CUTTERHEAD_RPM_MIN_MINING:.1f} rpm de consigna "
                    "(ejecute 'start_cutterhead')."
                )
            if self.bladder_pressure < BLADDER_P_MIN:
                return False, (
                    f"Vejiga sin agarre: P {self.bladder_pressure:.2f} bar < "
                    f"P_min {BLADDER_P_MIN:.2f} bar (ejecute 'inflate_bladder' y espere)."
                )
        return True, ""

    # --- Cabezal de corte ----------------------------------------------

    def _cmd_start_cutterhead(self, params):
        if self.state not in (State.READY, State.MINING):
            return False, (
                "El cabezal de corte solo puede arrancar en READY o MINING "
                f"(estado actual {self.state.value})."
            )
        if self._inhibited(SUB_CUTTERHEAD):
            return False, "Cabezal inhabilitado por un interlock enganchado."
        rpm = float(params.get("rpm", self.cutterhead_rpm_target or CUTTERHEAD_RPM_DEFAULT))
        self.cutterhead_rpm_target = clamp(rpm, 0.5, CUTTERHEAD_RPM_MAX)
        self.cutterhead_running = True
        return True, f"Cabezal en marcha, consigna {self.cutterhead_rpm_target:.2f} rpm."

    def _cmd_stop_cutterhead(self, params):
        was_mining = self.state is State.MINING
        self.cutterhead_running = False
        self.cutterhead_rpm_target = 0.0
        if was_mining:
            self._set_state(State.READY, "cabezal de corte detenido por el operador")
            return True, "Cabezal detenido; excavacion suspendida y maquina en READY."
        return True, "Cabezal de corte detenido."

    def _cmd_set_cutterhead_rpm(self, params):
        if "rpm" not in params:
            return False, "Falta el parametro 'rpm'."
        if self.state not in (State.READY, State.MINING):
            return False, (
                f"Solo se ajustan rpm en READY o MINING (estado actual {self.state.value})."
            )
        rpm = clamp(float(params["rpm"]), 0.0, CUTTERHEAD_RPM_MAX)
        self.cutterhead_rpm_target = rpm
        if rpm <= 0.0:
            return self._cmd_stop_cutterhead({})
        self.cutterhead_running = True
        if self.state is State.MINING and rpm < CUTTERHEAD_RPM_MIN_MINING:
            self._set_state(State.READY, "rpm del cabezal por debajo del minimo para excavar")
            return True, (
                f"Consigna {rpm:.2f} rpm por debajo del minimo para excavar; maquina en READY."
            )
        return True, f"Consigna del cabezal en {rpm:.2f} rpm."

    # --- Direccion A/B --------------------------------------------------

    def _cmd_steer(self, params):
        motor = str(params.get("motor", "A")).strip().upper()
        if motor not in ("A", "B", "BOTH"):
            return False, "El parametro 'motor' debe ser 'A', 'B' o 'both'."
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return False, (
                f"Los motores de direccion no operan en {self.state.value}."
            )
        if self._inhibited(SUB_STEERING):
            return False, "Direccion inhabilitada por un interlock enganchado."

        if "position_mm" in params:
            requested = float(params["position_mm"])
            absolute = True
        elif "delta_mm" in params:
            requested = float(params["delta_mm"])
            absolute = False
        else:
            return False, "Indique 'position_mm' (absoluto) o 'delta_mm' (relativo)."

        targets = ("a", "b") if motor == "BOTH" else (motor.lower(),)
        applied = []
        for axis in targets:
            base = 0.0 if absolute else getattr(self, f"steer_{axis}_target_mm")
            value = clamp(base + requested, -STEER_STROKE_MM, STEER_STROKE_MM)
            setattr(self, f"steer_{axis}_target_mm", value)
            applied.append(f"{axis.upper()}={value:+.2f} mm")
        return True, (
            "Consigna de direccion actualizada: " + ", ".join(applied)
            + f" (recorrido +/-{STEER_STROKE_MM:.0f} mm)."
        )

    def _cmd_center_steering(self, params):
        if self._inhibited(SUB_STEERING):
            return False, "Direccion inhabilitada por un interlock enganchado."
        self.steer_a_target_mm = 0.0
        self.steer_b_target_mm = 0.0
        return True, "Motores de direccion A y B centrados."

    # --- Propulsion y vejiga -------------------------------------------

    def _cmd_set_propulsion_speed(self, params):
        key = "speed_mm_min" if "speed_mm_min" in params else "speed"
        if key not in params:
            return False, "Falta el parametro 'speed_mm_min'."
        if self.state not in (State.READY, State.MINING):
            return False, (
                f"La propulsion solo se ajusta en READY o MINING (estado {self.state.value})."
            )
        speed = clamp(float(params[key]), 0.0, PROPULSION_SPEED_MAX)
        self.propulsion_speed_target = speed
        return True, f"Consigna de propulsion en {speed:.2f} mm/min."

    def _cmd_inflate_bladder(self, params):
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return False, f"No se puede inflar la vejiga en {self.state.value}."
        if self._inhibited(SUB_INFLATION):
            return False, "Inflado inhabilitado por un interlock enganchado."
        target = float(params.get("target_bar", BLADDER_P_TARGET_DEFAULT))
        self.bladder_target = clamp(target, 0.0, BLADDER_P_TARGET_MAX)
        self.bladder_inflating = True
        return True, (
            f"Inflado activo, consigna {self.bladder_target:.2f} bar "
            f"(rango seguro {BLADDER_P_MIN:.2f}-{BLADDER_P_MAX:.2f} bar)."
        )

    def _cmd_deflate_bladder(self, params):
        was_mining = self.state is State.MINING
        self.bladder_inflating = False
        self.bladder_target = 0.0
        self._grip_timer = 0.0
        if was_mining:
            self._set_state(State.READY, "vejiga despresurizada por el operador")
            return True, "Vejiga despresurizada; excavacion suspendida y maquina en READY."
        return True, "Vejiga despresurizada."

    # --- Simulacion de fallas y latido ---------------------------------

    def _cmd_simulate_fault(self, params):
        raw = str(params.get("fault", params.get("type", "generic"))).strip().lower()
        aliases = {
            "overload": "cutterhead_overload",
            "cutterhead": "cutterhead_overload",
            "cutterhead_overload": "cutterhead_overload",
            "underpressure": "bladder_underpressure",
            "leak": "bladder_underpressure",
            "bladder_underpressure": "bladder_underpressure",
            "overpressure": "bladder_overpressure",
            "bladder_overpressure": "bladder_overpressure",
            "comms": "comms_loss",
            "network": "comms_loss",
            "comms_loss": "comms_loss",
            "generic": "generic",
            "critical": "generic",
        }
        fault = aliases.get(raw)
        if fault is None:
            return False, (
                f"Falla desconocida '{raw}'. Validas: cutterhead_overload, "
                "bladder_underpressure, bladder_overpressure, comms_loss, generic."
            )

        if fault == "cutterhead_overload":
            self.forced_overload = True
            log_interlock("Falla inyectada: sobrecarga en el cabezal de corte.")
            note = "" if self.cutterhead_rpm > 0 else " (se manifestara al arrancar el cabezal)"
            return True, f"Sobrecarga inyectada; el interlock disparara solo{note}."

        if fault == "bladder_underpressure":
            self.bladder_leak = True
            log_interlock("Falla inyectada: fuga en la vejiga de propulsion.")
            return True, "Fuga inyectada; la presion caera por debajo de P_min."

        if fault == "bladder_overpressure":
            self.pump_stuck = True
            log_interlock("Falla inyectada: valvula de inflado trabada (sobrepresion).")
            return True, "Bomba trabada inyectada; la presion superara P_max."

        if fault == "comms_loss":
            self.forced_comms_loss = True
            log_interlock("Falla inyectada: perdida de comunicacion de red.")
            return True, "Perdida de red simulada; se ejecutara el aborto global."

        if self.state in (State.POWER_OFF, State.E_STOPPED):
            return False, f"No hay actuacion que abortar en {self.state.value}."
        reason = str(params.get("reason", "falla critica simulada por el operador"))
        log_interlock(f"Falla critica generica inyectada: {reason}")
        self._set_state(State.FAULT, reason)
        return True, "Maquina en FAULT. Use 'clear_fault' + 'arm' para recuperarla."

    def _cmd_heartbeat(self, params):
        return True, "Latido recibido; watchdog de comunicaciones refrescado."

    # --- Uplink de telemetria ------------------------------------------

    def register_uplink_result(self, ok, detail=""):
        with self._lock:
            if ok:
                self.uplink_ok = True
                self.uplink_failures = 0
                self.last_contact_ts = time.time()
                return
            self.uplink_ok = False
            self.uplink_failures += 1
            if self.uplink_failures == UPLINK_MAX_FAILURES and not self.forced_comms_loss:
                log_net(
                    f"{self.uplink_failures} envios de telemetria fallidos consecutivos "
                    f"({detail}): se declara perdida de comunicacion."
                )
                self.forced_comms_loss = True


# =========================================================================
# 5. DOCUMENTACION DE LA API (se sirve en GET /)
# =========================================================================

COMMAND_HELP = {
    "power_on": "Aplica la energia principal. Paso 1/2 del reinicio seguro desde POWER OFF.",
    "power_off": "Corta la energia y detiene toda la actuacion. Lleva a POWER OFF.",
    "estop": "Parada de emergencia inmediata. Params opcionales: {'reason': texto}.",
    "reset_estop": "Libera el hongo de emergencia. Paso 1/2 del reinicio desde E-STOPPED.",
    "clear_fault": "Resetea los interlocks enganchados. Paso 1/2 del reinicio desde FAULT.",
    "arm": "Confirma el rearme. Paso 2/2 obligatorio de todo reinicio seguro; lleva a IDLE.",
    "set_state": "Solicita un cambio de estado. Params: {'state': 'IDLE|READY|MINING|...'}.",
    "start_cutterhead": "Arranca el cabezal de corte. Params opcionales: {'rpm': 3.5}.",
    "stop_cutterhead": "Detiene el cabezal de corte (y suspende la excavacion).",
    "set_cutterhead_rpm": "Ajusta la consigna de giro. Params: {'rpm': 0-8}.",
    "steer": "Mueve los motores de direccion. Params: {'motor': 'A|B|both', "
             "'position_mm': -25..25} o {'delta_mm': x}.",
    "center_steering": "Devuelve los motores de direccion A y B al centro.",
    "set_propulsion_speed": "Consigna de avance de las orugas. Params: {'speed_mm_min': 0-60}.",
    "inflate_bladder": "Presuriza la vejiga de propulsion. Params: {'target_bar': 0-2.6}.",
    "deflate_bladder": "Despresuriza la vejiga (suspende la excavacion).",
    "simulate_fault": "Inyecta una falla para probar alarmas. Params: {'fault': "
                      "'cutterhead_overload|bladder_underpressure|bladder_overpressure|"
                      "comms_loss|generic'}.",
    "heartbeat": "Refresca el watchdog de comunicaciones sin alterar la maquina.",
}


def api_help():
    return {
        "service": "B.I.C.H.O. Digital Twin",
        "team": TEAM_NAME,
        "version": VERSION,
        "endpoints": {
            "GET /": "Esta ayuda.",
            "GET /health": "Resumen corto de estado y seguridad.",
            "GET /telemetry": "Telemetria completa en el formato exigido por TBC.",
            "POST /command": "Modo manual/ingenieria: {'command': nombre, 'params': {...}}.",
            "POST /tbc_endpoint": "Receptor simulado de telemetria de The Boring Company.",
        },
        "states": [state.value for state in State],
        "allowed_transitions": {
            state.value: sorted(target.value for target in targets)
            for state, targets in ALLOWED_TRANSITIONS.items()
        },
        "safe_restart": {
            state.value: {"independent_actions_required": len(sequence), "sequence": list(sequence)}
            for state, sequence in RESTART_SEQUENCES.items()
        },
        "interlocks": {
            "cutterhead_overload": f"Carga > {CUTTERHEAD_LOAD_MAX_PCT:.0f} % o corriente > "
                                   f"{CUTTERHEAD_CURRENT_MAX_A:.0f} A durante "
                                   f"{OVERLOAD_DEBOUNCE_S:.1f} s: detiene cabezal y propulsion.",
            "bladder_underpressure": f"P_vejiga < {BLADDER_P_MIN:.2f} bar: deshabilita la propulsion.",
            "bladder_overpressure": f"P_vejiga > {BLADDER_P_MAX:.2f} bar: detiene el inflado, "
                                    "abre el alivio y deshabilita la propulsion.",
            "comms_loss": f"Sin trafico durante mas de {COMMS_TIMEOUT_S:.0f} s en READY/MINING: "
                          "aborto global de la actuacion.",
        },
        "commands": COMMAND_HELP,
        "startup_sequence": [
            "power_on", "arm", "set_state READY", "inflate_bladder",
            "start_cutterhead", "set_state MINING",
        ],
    }


# =========================================================================
# 6. SERVIDOR HTTP
# =========================================================================

class BichoRequestHandler(BaseHTTPRequestHandler):
    """Handler REST del gemelo digital."""

    server_version = f"BICHO-DigitalTwin/{VERSION}"
    protocol_version = "HTTP/1.1"
    simulator = None                 # inyectado en main()

    # --- Helpers -------------------------------------------------------

    def _send_json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _drain_body(self):
        """Consume el cuerpo para no desincronizar una conexion keep-alive."""
        try:
            remaining = min(int(self.headers.get("Content-Length") or 0), 1_000_000)
        except ValueError:
            remaining = 0
        while remaining > 0:
            chunk = self.rfile.read(min(65536, remaining))
            if not chunk:
                break
            remaining -= len(chunk)

    def _read_json_body(self):
        """Devuelve (dict, None) o (None, mensaje_de_error)."""
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self.close_connection = True
            return None, "Cabecera Content-Length invalida."
        if length <= 0:
            return None, "Cuerpo vacio: se espera un objeto JSON."
        if length > 1_000_000:
            self.close_connection = True
            return None, "Cuerpo demasiado grande."
        raw = self.rfile.read(length)
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return None, f"JSON invalido: {exc}"
        if not isinstance(body, dict):
            return None, "El JSON debe ser un objeto, por ejemplo {\"command\": \"arm\"}."
        return body, None

    @property
    def route(self):
        return self.path.split("?", 1)[0].rstrip("/") or "/"

    # --- Verbos HTTP ---------------------------------------------------

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Max-Age", "86400")
        self.end_headers()

    def do_GET(self):
        sim = self.simulator
        route = self.route

        if route == "/telemetry":
            sim.touch_comms()
            self._send_json(200, sim.telemetry())
        elif route in ("/", "/help"):
            self._send_json(200, api_help())
        elif route in ("/health", "/status", "/state"):
            self._send_json(200, sim.status_summary())
        else:
            self._send_json(404, {
                "ok": False,
                "error": f"Ruta no encontrada: {self.path}",
                "hint": "Use GET /telemetry, GET / o POST /command.",
            })

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        sim = self.simulator
        route = self.route

        if route == "/command":
            body, error = self._read_json_body()
            if error:
                log_warn(f"POST /command rechazado: {error}")
                self._send_json(400, {"ok": False, "error": error, "commands": sorted(COMMAND_HELP)})
                return
            command = body.get("command", body.get("cmd", body.get("action")))
            if command is None:
                self._send_json(400, {
                    "ok": False,
                    "error": "Falta la llave 'command'.",
                    "example": {"command": "set_state", "params": {"state": "READY"}},
                    "commands": sorted(COMMAND_HELP),
                })
                return
            params = body.get("params")
            if not isinstance(params, dict):
                params = {
                    key: value
                    for key, value in body.items()
                    if key not in ("command", "cmd", "action", "params")
                }
            code, payload = sim.execute(command, params)
            self._send_json(code, payload)
            return

        if route == "/tbc_endpoint":
            body, error = self._read_json_body()
            if error:
                self._send_json(400, {"status": "error", "message": error})
                return
            sim.touch_comms()
            log_net(
                "📦 Paquete de telemetria recibido por el receptor TBC simulado "
                f"(team={body.get('team')}, chainage={body.get('chainage')}, "
                f"mining={body.get('mining')})."
            )
            self._send_json(200, {"status": "success", "message": "Telemetry received by TBC"})
            return

        self._drain_body()
        self._send_json(404, {"ok": False, "error": f"Ruta no encontrada: {self.path}"})

    def log_message(self, fmt, *args):
        """Silencia el log de accesos de http.server (usamos el nuestro)."""
        return


# =========================================================================
# 7. HILOS DE FONDO
# =========================================================================

class SimulationLoop(threading.Thread):
    """Ejecuta la fisica del gemelo digital a TICK_HZ."""

    def __init__(self, simulator, stop_event):
        super().__init__(name="sim-loop", daemon=True)
        self.simulator = simulator
        self.stop_event = stop_event

    def run(self):
        period = 1.0 / TICK_HZ
        last = time.perf_counter()
        while not self.stop_event.is_set():
            now = time.perf_counter()
            dt = clamp(now - last, 0.0, 0.5)
            last = now
            try:
                self.simulator.tick(dt)
            except Exception as exc:                       # el lazo nunca debe morir
                log_warn(f"Error en el lazo de simulacion: {exc!r}")
            self.stop_event.wait(period)


class TelemetryUplink(threading.Thread):
    """Envia la telemetria por HTTP POST (cumplimiento >= 0.1 Hz)."""

    def __init__(self, simulator, url, hz, stop_event, token=None):
        super().__init__(name="telemetry-uplink", daemon=True)
        self.simulator = simulator
        self.url = url
        self.period = 1.0 / max(0.01, hz)
        self.stop_event = stop_event
        self.token = token

    def run(self):
        log_net(f"Uplink de telemetria activo -> {self.url} cada {self.period:.1f} s.")
        while not self.stop_event.is_set():
            payload = json.dumps(self.simulator.telemetry()).encode("utf-8")
            request = urllib.request.Request(
                self.url,
                data=payload,
                method="POST",
                headers={"Content-Type": "application/json"},
            )
            if self.token:
                request.add_header("Authorization", f"Bearer {self.token}")
            try:
                with urllib.request.urlopen(request, timeout=5.0) as response:
                    ok = 200 <= response.status < 300
                    self.simulator.register_uplink_result(ok, f"HTTP {response.status}")
            except (urllib.error.URLError, OSError, ValueError) as exc:
                self.simulator.register_uplink_result(False, repr(exc))
            self.stop_event.wait(self.period)


# =========================================================================
# 8. ARRANQUE
# =========================================================================

def print_banner(host, port, simulator, uplink_url):
    shown_host = host or "0.0.0.0"
    base = f"http://localhost:{port}"
    print("=" * 72)
    print(f"🚀  GEMELO DIGITAL B.I.C.H.O.  v{VERSION}   |   {TEAM_NAME}")
    print("=" * 72)
    print(f"🌐  Escuchando en {shown_host}:{port}   (lazo fisico a {TICK_HZ:.0f} Hz)")
    print(f"📡  Telemetria .............. GET  {base}/telemetry")
    print(f"🎛️   Modo manual/ingenieria .. POST {base}/command")
    print(f"🛑  Receptor TBC simulado ... POST {base}/tbc_endpoint")
    print(f"📖  Ayuda de la API ......... GET  {base}/")
    if uplink_url:
        print(f"⬆️   Uplink de telemetria .... POST {uplink_url}")
    print("-" * 72)
    print(f"🔐  Estado inicial: {simulator.state.value}")
    print("    Secuencia de puesta en marcha (cada paso es un POST /command):")
    print("      1) power_on          2) arm                 -> IDLE")
    print("      3) set_state READY   4) inflate_bladder     -> vejiga con agarre")
    print("      5) start_cutterhead  6) set_state MINING    -> excavando")
    print("    Reinicio seguro: E-STOPPED / POWER OFF / FAULT exigen 2 acciones.")
    print("-" * 72)
    print("    Ejemplo (PowerShell):")
    print(f"      Invoke-RestMethod {base}/command -Method Post "
          "-ContentType 'application/json' -Body '{\"command\":\"power_on\"}'")
    print("    Ejemplo (curl):")
    print(f"      curl -X POST {base}/command -H \"Content-Type: application/json\" "
          "-d \"{\\\"command\\\":\\\"estop\\\"}\"")
    print("-" * 72)
    print("⚠️   Ctrl+C para apagar el gemelo digital.")
    print("=" * 72, flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Backend simulado (gemelo digital) del B.I.C.H.O. - UDD Tunnel Lab."
    )
    parser.add_argument("--host", default="", help="Interfaz de escucha (por defecto: todas).")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Puerto HTTP.")
    parser.add_argument(
        "--start-state",
        default="POWER OFF",
        help="Estado inicial (POWER OFF por defecto; use IDLE para desarrollo de GUI).",
    )
    parser.add_argument(
        "--comms-timeout",
        type=float,
        default=COMMS_TIMEOUT_S,
        help="Segundos sin trafico antes del interlock de perdida de red.",
    )
    parser.add_argument("--uplink-url", default=None, help="URL destino del uplink de telemetria.")
    parser.add_argument(
        "--uplink-hz", type=float, default=1.0, help="Frecuencia del uplink (minimo TBC: 0.1 Hz)."
    )
    parser.add_argument("--uplink-token", default=None, help="Bearer token para el uplink.")
    parser.add_argument("--seed", type=int, default=None, help="Semilla del ruido simulado.")
    return parser.parse_args(argv)


def main(argv=None):
    configure_console()
    args = parse_args(argv)

    if args.seed is not None:
        random.seed(args.seed)

    start_state = parse_state(args.start_state)
    if start_state is None:
        print(f"Estado inicial invalido: '{args.start_state}'. "
              f"Validos: {', '.join(s.value for s in State)}")
        return 2
    if start_state in (State.MINING, State.FAULT, State.E_STOPPED):
        print(f"El estado inicial no puede ser {start_state.value}: la maquina debe "
              "arrancar en POWER OFF, IDLE o READY.")
        return 2
    if args.uplink_hz < 0.1:
        print("El uplink no puede ser mas lento que 0.1 Hz (requisito TBC).")
        return 2

    simulator = BichoSimulator(comms_timeout=args.comms_timeout, start_state=start_state)
    BichoRequestHandler.simulator = simulator

    stop_event = threading.Event()
    SimulationLoop(simulator, stop_event).start()
    if args.uplink_url:
        TelemetryUplink(
            simulator, args.uplink_url, args.uplink_hz, stop_event, args.uplink_token
        ).start()

    try:
        server = ThreadingHTTPServer((args.host, args.port), BichoRequestHandler)
    except OSError as exc:
        stop_event.set()
        print(f"No se pudo abrir el puerto {args.port}: {exc}")
        return 1
    server.daemon_threads = True

    print_banner(args.host, args.port, simulator, args.uplink_url)

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print()
        log_info("Apagado solicitado por el operador (Ctrl+C).")
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        telemetry = simulator.telemetry()
        log_info(
            f"Resumen final: estado={telemetry['extra']['state']}, "
            f"chainage={telemetry['chainage']} m, "
            f"comandos={telemetry['extra']['sim']['commands_received']}, "
            f"tiempo excavando={telemetry['extra']['navigation']['mining_time_s']} s."
        )
        print("🛑 Gemelo digital B.I.C.H.O. detenido.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
