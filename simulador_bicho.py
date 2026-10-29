#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=========================================================================
 B.I.C.H.O. - Controlador Supervisor / Gemelo Digital   |   UDD Tunnel Lab
=========================================================================
Controlador supervisor de la tuneladora a escala del equipo UDD Tunnel Lab
para la Not-a-Boring Competition (The Boring Company, TBC).

Arquitectura (Clean Architecture + Inversion de Dependencias, PRD v2.0):

    +-------------------------------------------------------------------+
    | CAPA 4  PRESENTACION / API   aiohttp: GET /telemetry, POST        |
    |         /command, POST /api/plc/parada + cliente POST hacia TBC   |
    +----------------------------+--------------------------------------+
                                 | depende de
    +----------------------------v--------------------------------------+
    | CAPA 3  NUCLEO / DOMINIO     TBMController: maquina de estados,   |
    |         interlocks PDB 5.6, Safe Restart, watchdog, lazo 10 Hz    |
    +----------------------------+--------------------------------------+
                                 | depende de (abstraccion)
    +----------------------------v--------------------------------------+
    | CAPA 1  PUERTOS              IHardwareAdapter (contrato)          |
    +----------------------------^--------------------------------------+
                                 | implementa
    +----------------------------+--------------------------------------+
    | CAPA 2  ADAPTADORES          SimulatedHardwareAdapter (planta     |
    |         simulada: ruido blanco, 6DOF, sensor de vejiga)           |
    +-------------------------------------------------------------------+

    Regla de dependencias: Presentacion -> Nucleo -> Puertos <- Adaptadores.
    El nucleo NUNCA importa un adaptador concreto: recibe el hardware por
    inyeccion de dependencias en `main()` (raiz de composicion). Cambiar de
    simulador a un PLC Siemens / servos SEW = escribir otra clase que herede
    de `IHardwareAdapter` y pasarla al `TBMController`; nada mas cambia.

Seguridad (reglamento TBC y PDB del equipo):
  * Estados: POWER OFF / E-STOPPED / IDLE / READY / MINING / FAULT con
    transiciones validadas (nunca se pasa de un estado inseguro a excavar).
  * Safe Restart (TBC 10.g): salir de E-STOPPED, POWER OFF o FAULT exige
    SIEMPRE 2 acciones operador independientes y en orden.
  * Interlocks (PDB 5.6): sobrecarga del cabezal, sobrecarga de la
    propulsion, sobrepresion de vejiga (alivio), perdida de presion de
    vejiga, sensor inalambrico de vejiga y perdida de red.
  * Modos de mando (PDB 6.6.2): ningun comando manual puede saltarse un
    interlock; la compuerta de seguridad se aplica en la etapa de salida.
  * Watchdog de red de 10 s: un enlace armado sin vida > 10 s => FAULT.
  * E-Stop: actuadores a 0 de inmediato, pero `read_sensors()` se sigue
    llamando a 10 Hz y la telemetria sigue viva.
  * Parada externa (HMI/PLC): `POST /api/plc/parada` -> `engage_estop()`
    fuerza E-STOPPED sin validar transiciones y sin rechazarse por un
    payload invalido (fail-safe); no toca interlocks y salir exige igual
    'reset_estop' + 'arm'.
  * Cliente TBC (Regla 7): POST de la telemetria a >= 0.1 Hz en segundo plano.

Dependencia externa unica: aiohttp  ->  pip install -r requirements.txt

Uso:
    python simulador_bicho.py
    python simulador_bicho.py --port 8000 --start-state IDLE
    python simulador_bicho.py --uplink-url https://servidor-tbc/endpoint --uplink-hz 0.5
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Optional

try:                                   # Solo la capa de presentacion lo usa.
    import aiohttp
    from aiohttp import web
except ImportError:                    # pragma: no cover
    aiohttp = None
    web = None


# =========================================================================
# 0. NUCLEO COMPARTIDO (identidad, especificacion de la maquina, logging)
#    Sin dependencias hacia ninguna capa: lo pueden usar todas.
# =========================================================================

TEAM_NAME = "UDD Tunnel Lab"
VERSION = "3.0"

TICK_HZ = 10.0                      # lazo de control y muestreo: 10 Hz
TICK_PERIOD_S = 1.0 / TICK_HZ
DEFAULT_PORT = 8000

# Limites nominales de la maquina (los comparten nucleo y planta).
CUTTERHEAD_RPM_MAX = 8.0
PROPULSION_SPEED_MAX = 60.0         # mm/min
STEER_STROKE_MM = 25.0              # recorrido +/- respecto al centro


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
    _log("⚠️ ", "WARN", message)


def log_command(message):
    _log("📥", "COMMAND", message)


def log_state(message):
    _log("🔄", "STATE", message)


def log_interlock(message):
    _log("🚨", "INTERLOCK", message)


def log_net(message):
    _log("📡", "NET", message)


def configure_console():
    """Evita UnicodeEncodeError al redirigir la salida en Windows."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


# =========================================================================
# CAPA 1 - PUERTOS (interfaces hacia el hardware)
# =========================================================================
# Contrato de datos entre el nucleo y cualquier hardware. Un adaptador real
# (PLC Siemens, servos SEW, ...) debe entregar y aceptar exactamente estas
# claves; el nucleo las valida en cada ciclo.

SENSOR_KEYS = (
    # --- Navegacion 6DOF medida (puede traer ruido) + odometria ---
    "chainage_m", "easting_m", "northing_m", "elevation_m",
    "roll_deg", "pitch_deg", "heading_deg",
    "chainage_odometry_m",          # avance integrado por encoders de oruga
    "yaw_rate_deg_s",
    "ground_hardness_index",
    # --- Cabezal de corte ---
    "cutterhead_rpm", "cutterhead_load_pct", "cutterhead_current_a",
    "cutterhead_torque_nm", "cutterhead_temp_c",
    # --- Propulsion tipo oruga ---
    "propulsion_speed_mm_min", "track_left_mm_min", "track_right_mm_min",
    "thrust_force_kn", "propulsion_load_pct", "propulsion_current_a",
    "propulsion_temp_c",
    # --- Vejiga e hydroshield ---
    "bladder_pressure_bar", "hydroshield_pressure_bar",
    # --- Motores de direccion A (guiñada) y B (cabeceo) ---
    "steer_a_mm", "steer_b_mm",
    # --- Sensor inalambrico de presion de vejiga ---
    "bladder_sensor_battery_pct", "bladder_sensor_signal_ms",
    "bladder_sensor_link_ok",
)

ACTUATOR_KEYS = (
    "safe_torque_off",                  # True => corte inmediato de accionamientos
    "main_power",                       # False => planta sin energia
    "advance_enabled",                  # autorizacion de avance (solo en MINING)
    "cutterhead_rpm_setpoint",
    "propulsion_speed_setpoint_mm_min",
    "bladder_inflate_enable",
    "bladder_pressure_setpoint_bar",
    "relief_valve_open",
    "steer_a_setpoint_mm",
    "steer_b_setpoint_mm",
    "hydroshield_setpoint_bar",
)


def safe_actuator_commands():
    """Comandos de estado seguro: sin movimiento, sin energia de accionamientos."""
    return {
        "safe_torque_off": True,
        "main_power": False,
        "advance_enabled": False,
        "cutterhead_rpm_setpoint": 0.0,
        "propulsion_speed_setpoint_mm_min": 0.0,
        "bladder_inflate_enable": False,
        "bladder_pressure_setpoint_bar": 0.0,
        "relief_valve_open": False,
        "steer_a_setpoint_mm": 0.0,
        "steer_b_setpoint_mm": 0.0,
        "hydroshield_setpoint_bar": 0.0,
    }


class HardwareError(RuntimeError):
    """Error de E/S entre el nucleo y el hardware (contrato violado o enlace roto)."""


class IHardwareAdapter(ABC):
    """Puerto de hardware: contrato que todo hardware debe cumplir.

    El nucleo (`TBMController`) solo conoce esta interfaz. El hardware se
    trata como una "caja negra" intercambiable.

    Reglas del contrato:
      * Los 4 metodos deben ser NO BLOQUEANTES (sub-milisegundo): el lazo de
        control corre a 10 Hz dentro del event loop. Un adaptador con E/S
        bloqueante (Modbus, S7, CAN...) debe tener su propio hilo de sondeo
        y devolver aqui el ultimo valor cacheado.
      * `read_sensors()` devuelve un dict con TODAS las claves de
        `SENSOR_KEYS`; `write_actuators()` recibe TODAS las de `ACTUATOR_KEYS`.
      * Cualquier excepcion se interpreta como perdida de enlace con el
        hardware y el nucleo pasa a estado seguro.
    """

    @abstractmethod
    def read_sensors(self) -> dict:
        """Lee los sensores (claves `SENSOR_KEYS`)."""

    @abstractmethod
    def write_actuators(self, commands: dict) -> None:
        """Escribe las consignas de actuadores (claves `ACTUATOR_KEYS`)."""

    @abstractmethod
    def check_network_health(self) -> bool:
        """True si la red/bus de campo con el hardware esta sana."""

    @abstractmethod
    def is_hardware_estop_triggered(self) -> bool:
        """True mientras el hongo de emergencia FISICO este presionado."""


class IFaultInjectionPort(ABC):
    """Puerto opcional (segregacion de interfaces) solo para hardware simulado.

    Permite inyectar fallas de prueba desde `POST /command`. Un adaptador de
    hardware real NO lo implementa y el nucleo rechaza esos comandos.
    """

    @abstractmethod
    def available_faults(self) -> list:
        """Nombres de las fallas inyectables."""

    @abstractmethod
    def inject_fault(self, fault: str, params: Mapping) -> tuple:
        """Inyecta una falla. Devuelve (ok, mensaje)."""

    @abstractmethod
    def clear_injected_faults(self) -> None:
        """Retira todas las fallas inyectadas y restaura el sensor inalambrico."""

    @abstractmethod
    def injected_faults(self) -> list:
        """Nombres de las fallas actualmente inyectadas."""

    @abstractmethod
    def set_wireless_sensor(self, params: Mapping) -> tuple:
        """Ajusta bateria/señal del sensor inalambrico. Devuelve (ok, mensaje)."""

    @abstractmethod
    def set_hardware_estop(self, pressed: bool) -> None:
        """Presiona o libera el hongo de emergencia fisico simulado."""


# =========================================================================
# CAPA 2 - ADAPTADORES (hardware simulado)
# =========================================================================
# Todo lo "fisico" vive aqui: dinamica de actuadores, ruido blanco, evolucion
# 6DOF, termica, terreno y degradacion del sensor inalambrico de vejiga.
# Esta capa solo depende del puerto: no sabe nada de estados ni interlocks.

# --- Cabezal de corte ---
CUTTERHEAD_RPM_RAMP = 1.2           # rpm/s acelerando
CUTTERHEAD_RPM_BRAKE = 3.0          # rpm/s frenando
CUTTERHEAD_AMPS_PER_PCT = 0.32
CUTTERHEAD_TORQUE_MAX_NM = 900.0
CUTTERHEAD_FAULT_LOAD_PCT = 55.0    # carga extra de la falla inyectada

# --- Propulsion tipo oruga ---
PROPULSION_RAMP = 12.0              # mm/min por segundo
PROPULSION_AMPS_PER_PCT = 0.24
PROPULSION_FAULT_LOAD_PCT = 45.0    # carga extra de la falla inyectada

# --- Vejiga de propulsion ---
BLADDER_INFLATE_RATE = 0.40         # bar/s de inflado
BLADDER_LEAK_RATE = 0.015           # bar/s de fuga normal
BLADDER_LEAK_FAULT_RATE = 0.55      # bar/s de fuga simulada (falla inyectada)
BLADDER_PUMP_STUCK_RATE = 0.50      # bar/s con bomba trabada (falla inyectada)
BLADDER_RELIEF_RATE = 0.90          # bar/s de la valvula de alivio
BLADDER_UNPOWERED_LEAK_RATE = 0.10  # bar/s: sin energia la vejiga se despresuriza

# --- Motores de direccion A (guiñada) y B (cabeceo) ---
STEER_RATE_MM_S = 2.0
STEER_YAW_DEG_PER_M = 0.060         # grados de guiñada por mm de A y metro avanzado
STEER_PITCH_DEG_PER_M = 0.040       # grados de cabeceo por mm de B y metro avanzado
PITCH_LIMIT_DEG = 8.0

# --- Sensor inalambrico de vejiga ---
BLADDER_SENSOR_BATTERY_START = 100.0
BLADDER_SENSOR_DRAIN_PCT_PER_H = 0.8        # descarga nominal (horas de autonomia)
BLADDER_SENSOR_DRAIN_FAULT_PCT_S = 6.0      # descarga acelerada para pruebas
BLADDER_SENSOR_SIGNAL_BASE_MS = 18.0
BLADDER_SENSOR_LINK_LOSS_MS = 250.0         # el radio da el enlace por perdido
BLADDER_SENSOR_SIGNAL_TIMEOUT_MS = 999.0    # valor publicado cuando no hay enlace

# --- Ruido blanco de sensores (1-sigma) para probar filtros ---
NOISE_PRESSURE_BAR = 0.012
NOISE_POSE_M = 0.004
NOISE_ATTITUDE_DEG = 0.035
NOISE_TEMP_C = 0.08
NOISE_LOAD_PCT = 0.35
NOISE_CURRENT_A = 0.08
NOISE_SIGNAL_MS = 2.4
NOISE_BATTERY_PCT = 0.04

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

# --- Integracion numerica ---
MAX_PHYSICS_STEP_S = 0.1            # paso maximo de integracion
MAX_CATCHUP_S = 0.5                 # tope de tiempo a recuperar en una lectura

# Alias de las fallas inyectables -> nombre canonico.
FAULT_ALIASES = {
    "overload": "cutterhead_overload",
    "cutterhead": "cutterhead_overload",
    "cutterhead_overload": "cutterhead_overload",
    "propulsion": "propulsion_overload",
    "propulsion_overload": "propulsion_overload",
    "underpressure": "bladder_underpressure",
    "leak": "bladder_underpressure",
    "bladder_underpressure": "bladder_underpressure",
    "overpressure": "bladder_overpressure",
    "bladder_overpressure": "bladder_overpressure",
    "comms": "comms_loss",
    "network": "comms_loss",
    "comms_loss": "comms_loss",
    "wireless": "bladder_sensor",
    "bladder_sensor": "bladder_sensor",
    "sensor": "bladder_sensor",
    "battery": "bladder_sensor_battery",
    "bladder_sensor_battery": "bladder_sensor_battery",
    "signal": "bladder_sensor_signal",
    "bladder_sensor_signal": "bladder_sensor_signal",
    "hardware_estop": "hardware_estop",
    "hw_estop": "hardware_estop",
}


class SimulatedHardwareAdapter(IHardwareAdapter, IFaultInjectionPort):
    """Hardware simulado para desarrollo y pruebas.

    Es una planta autonoma: avanza su propia fisica con el reloj cada vez
    que se leen los sensores (como lo haria un proceso real), obedece las
    consignas de `write_actuators()` y entrega lecturas con ruido blanco.
    """

    name = "SimulatedHardwareAdapter"

    def __init__(self, *, rng=None, clock=time.monotonic):
        self._rng = rng if rng is not None else random.Random()
        self._clock = clock
        self._last_t = clock()
        self._sim_time = 0.0

        # --- Navegacion 6DOF (verdad fisica) ---
        self._chainage = START_CHAINAGE_M
        self._easting = START_EASTING_M
        self._northing = START_NORTHING_M
        self._elevation = START_ELEVATION_M
        self._roll = START_ROLL_DEG
        self._pitch = START_PITCH_DEG
        self._heading = START_HEADING_DEG
        self._yaw_rate = 0.0
        self._vib_roll = 0.0
        self._vib_pitch = 0.0
        self._vib_phase = 0.0

        # --- Terreno ---
        self._ground_hardness = 1.0
        self._ground_phase = 0.0
        self._ground_walk = 0.0

        # --- Cabezal de corte ---
        self._ch_rpm = 0.0
        self._ch_load = 0.0
        self._ch_current = 0.0
        self._ch_torque = 0.0
        self._ch_temp = AMBIENT_TEMP_C

        # --- Propulsion ---
        self._prop_speed = 0.0
        self._track_left = 0.0
        self._track_right = 0.0
        self._thrust = 0.0
        self._prop_load = 0.0
        self._prop_current = 0.0
        self._prop_temp = AMBIENT_TEMP_C

        # --- Vejiga, hydroshield y direccion ---
        self._bladder_p = 0.0
        self._hydro_p = 0.0
        self._steer_a = 0.0
        self._steer_b = 0.0

        # --- Sensor inalambrico de vejiga (verdad fisica) ---
        self._bs_battery = BLADDER_SENSOR_BATTERY_START
        self._bs_signal_ms = BLADDER_SENSOR_SIGNAL_BASE_MS
        self._bs_link_ok = True

        # --- Fallas inyectadas y hongo fisico ---
        self._faults = set()
        self._hw_estop = False

        # Ultima consigna recibida (estado seguro hasta que el nucleo escriba).
        self._cmd = safe_actuator_commands()

    # -------------------------------------------------------------------
    # Contrato IHardwareAdapter
    # -------------------------------------------------------------------

    def read_sensors(self):
        self._advance_to_now()
        m = self._measure
        link_ok = self._bs_link_ok
        return {
            # Navegacion medida (con ruido blanco)
            "chainage_m": m(self._chainage, NOISE_POSE_M),
            "easting_m": m(self._easting, NOISE_POSE_M),
            "northing_m": m(self._northing, NOISE_POSE_M),
            "elevation_m": m(self._elevation, NOISE_POSE_M),
            "roll_deg": m(self._roll + self._vib_roll, NOISE_ATTITUDE_DEG),
            "pitch_deg": m(self._pitch + self._vib_pitch, NOISE_ATTITUDE_DEG),
            "heading_deg": m(self._heading, NOISE_ATTITUDE_DEG) % 360.0,
            # Variables sin ruido (encoders / calculadas)
            "chainage_odometry_m": self._chainage,
            "yaw_rate_deg_s": self._yaw_rate,
            "ground_hardness_index": self._ground_hardness,
            # Cabezal de corte
            "cutterhead_rpm": self._ch_rpm,
            "cutterhead_load_pct": m(self._ch_load, NOISE_LOAD_PCT, 0.0),
            "cutterhead_current_a": m(self._ch_current, NOISE_CURRENT_A, 0.0),
            "cutterhead_torque_nm": self._ch_torque,
            "cutterhead_temp_c": m(self._ch_temp, NOISE_TEMP_C),
            # Propulsion
            "propulsion_speed_mm_min": self._prop_speed,
            "track_left_mm_min": self._track_left,
            "track_right_mm_min": self._track_right,
            "thrust_force_kn": self._thrust,
            "propulsion_load_pct": m(self._prop_load, NOISE_LOAD_PCT, 0.0),
            "propulsion_current_a": m(self._prop_current, NOISE_CURRENT_A, 0.0),
            "propulsion_temp_c": m(self._prop_temp, NOISE_TEMP_C),
            # Presiones
            "bladder_pressure_bar": m(self._bladder_p, NOISE_PRESSURE_BAR, 0.0),
            "hydroshield_pressure_bar": m(self._hydro_p, NOISE_PRESSURE_BAR, 0.0),
            # Direccion
            "steer_a_mm": self._steer_a,
            "steer_b_mm": self._steer_b,
            # Sensor inalambrico de vejiga
            "bladder_sensor_battery_pct": m(self._bs_battery, NOISE_BATTERY_PCT, 0.0, 100.0),
            "bladder_sensor_signal_ms": (
                m(self._bs_signal_ms, NOISE_SIGNAL_MS, 1.0)
                if link_ok else BLADDER_SENSOR_SIGNAL_TIMEOUT_MS
            ),
            "bladder_sensor_link_ok": link_ok,
        }

    def write_actuators(self, commands):
        missing = [key for key in ACTUATOR_KEYS if key not in commands]
        if missing:
            raise HardwareError(f"write_actuators() missing keys {missing}")
        # Se integra la fisica con la consigna ANTERIOR hasta este instante
        # y recien entonces entra la nueva (como un scan de PLC).
        self._advance_to_now()
        self._cmd = {key: commands[key] for key in ACTUATOR_KEYS}
        if self._cmd["safe_torque_off"]:
            # STO cableado: los accionamientos caen sin esperar al siguiente ciclo.
            self._zero_drives()

    def check_network_health(self):
        return "comms_loss" not in self._faults

    def is_hardware_estop_triggered(self):
        return self._hw_estop

    # -------------------------------------------------------------------
    # Contrato IFaultInjectionPort
    # -------------------------------------------------------------------

    def available_faults(self):
        return sorted(set(FAULT_ALIASES.values()))

    def inject_fault(self, fault, params):
        raw = str(fault).strip().lower()
        canonical = FAULT_ALIASES.get(raw)
        if canonical is None:
            return False, (
                f"Unknown fault '{raw}'. Valid: "
                f"{', '.join(self.available_faults())}, generic."
            )

        if canonical == "cutterhead_overload":
            self._faults.add(canonical)
            log_interlock("Fault injected: cutterhead overload.")
            note = "" if self._ch_rpm > 0 else " (will manifest when the cutterhead starts)"
            return True, f"Overload injected; the interlock will trip on its own{note}."

        if canonical == "propulsion_overload":
            self._faults.add(canonical)
            log_interlock("Fault injected: propulsion motor overload.")
            note = "" if self._prop_speed > 0 else " (will manifest when advancing)"
            return True, f"Propulsion overload injected; the interlock will trip on its own{note}."

        if canonical == "bladder_underpressure":
            self._faults.add(canonical)
            log_interlock("Fault injected: propulsion bladder leak.")
            return True, "Leak injected; pressure will drop below P_min."

        if canonical == "bladder_overpressure":
            self._faults.add(canonical)
            log_interlock("Fault injected: inflation valve stuck (overpressure).")
            return True, "Stuck pump injected; pressure will exceed P_max."

        if canonical == "comms_loss":
            self._faults.add(canonical)
            log_interlock("Fault injected: network communication loss with the hardware.")
            return True, (
                "Simulated network loss; the watchdog will perform a global abort "
                "if it does not recover within its window."
            )

        if canonical == "bladder_sensor_battery":
            self._faults.add(canonical)
            log_interlock("Fault injected: accelerated battery drain on the wireless sensor.")
            return True, "Sensor battery will drain to the critical threshold and trigger FAULT."

        if canonical == "bladder_sensor_signal":
            self._faults.add(canonical)
            log_interlock("Fault injected: wireless bladder sensor link loss.")
            return True, "Sensor link lost; the interlock will drive the machine to FAULT."

        if canonical == "bladder_sensor":
            self._faults.add("bladder_sensor_signal")
            log_interlock("Fault injected: wireless bladder sensor (link + battery).")
            return True, "Wireless sensor out of service; the machine will go to FAULT."

        # hardware_estop
        self.set_hardware_estop(True)
        return True, "PHYSICAL E-stop button pressed (simulated)."

    def clear_injected_faults(self):
        # El hongo fisico NO se libera aqui: solo se libera con set_hardware_estop(False).
        self._faults.clear()
        if self._bs_battery < 40.0:
            self._bs_battery = BLADDER_SENSOR_BATTERY_START
        self._bs_link_ok = True
        self._bs_signal_ms = BLADDER_SENSOR_SIGNAL_BASE_MS

    def injected_faults(self):
        names = set(self._faults)
        if self._hw_estop:
            names.add("hardware_estop")
        return sorted(names)

    def set_wireless_sensor(self, params):
        if "battery_percent" in params:
            self._bs_battery = clamp(float(params["battery_percent"]), 0.0, 100.0)
            self._faults.discard("bladder_sensor_battery")
        if "signal_ms" in params:
            self._bs_signal_ms = max(0.0, float(params["signal_ms"]))
            lost = self._bs_signal_ms >= BLADDER_SENSOR_LINK_LOSS_MS
            self._set_fault("bladder_sensor_signal", lost)
            self._bs_link_ok = not lost
        if "link_ok" in params:
            lost = not bool(params["link_ok"])
            self._set_fault("bladder_sensor_signal", lost)
            self._bs_link_ok = not lost
            if lost:
                self._bs_signal_ms = BLADDER_SENSOR_SIGNAL_TIMEOUT_MS
        return True, (
            f"Wireless sensor: battery {self._bs_battery:.1f} %, "
            f"signal {self._bs_signal_ms:.1f} ms, "
            f"link={'ok' if self._bs_link_ok else 'lost'}."
        )

    def set_hardware_estop(self, pressed):
        self._hw_estop = bool(pressed)

    def _set_fault(self, name, active):
        if active:
            self._faults.add(name)
        else:
            self._faults.discard(name)

    # -------------------------------------------------------------------
    # Medicion con ruido blanco
    # -------------------------------------------------------------------

    def _measure(self, truth, sigma, low=None, high=None):
        """Lectura de sensor = verdad + ruido blanco (para probar filtros)."""
        value = truth + (self._rng.gauss(0.0, sigma) if sigma > 0.0 else 0.0)
        if low is not None:
            value = max(low, value)
        if high is not None:
            value = min(high, value)
        return value

    # -------------------------------------------------------------------
    # Fisica de la planta
    # -------------------------------------------------------------------

    def _advance_to_now(self):
        """Integra la fisica hasta el instante actual (pasos de <= 100 ms)."""
        now = self._clock()
        elapsed = clamp(now - self._last_t, 0.0, MAX_CATCHUP_S)
        self._last_t = now
        if elapsed <= 1e-9:
            return
        substeps = max(1, math.ceil(elapsed / MAX_PHYSICS_STEP_S - 1e-9))
        step = elapsed / substeps
        for _ in range(substeps):
            self._integrate(step)

    def _integrate(self, dt):
        self._sim_time += dt
        self._update_ground(dt)
        self._update_cutterhead(dt)
        self._update_bladder(dt)
        self._update_propulsion(dt)
        self._update_steering(dt)
        self._update_hydroshield(dt)
        self._update_navigation(dt)
        self._update_thermal(dt)
        self._update_wireless_sensor(dt)

    def _zero_drives(self):
        """Corte inmediato de cabezal y orugas. No toca presiones ni pose 6DOF."""
        self._ch_rpm = 0.0
        self._ch_load = 0.0
        self._ch_current = 0.0
        self._ch_torque = 0.0
        self._prop_speed = 0.0
        self._track_left = 0.0
        self._track_right = 0.0
        self._prop_load = 0.0
        self._prop_current = 0.0
        self._yaw_rate = 0.0

    def _update_ground(self, dt):
        """Dureza del terreno: deriva lenta + caminata aleatoria acotada."""
        self._ground_phase += dt * 0.05
        self._ground_walk = clamp(
            self._ground_walk + self._rng.uniform(-0.05, 0.05) * dt, -0.08, 0.08
        )
        self._ground_hardness = clamp(
            1.0 + 0.10 * math.sin(self._ground_phase) + self._ground_walk, 0.82, 1.18
        )

    def _update_cutterhead(self, dt):
        if self._cmd["safe_torque_off"]:
            self._zero_drives()
            return
        target = self._cmd["cutterhead_rpm_setpoint"]

        if self._ch_rpm < target:
            self._ch_rpm = min(target, self._ch_rpm + CUTTERHEAD_RPM_RAMP * dt)
        else:
            self._ch_rpm = max(target, self._ch_rpm - CUTTERHEAD_RPM_BRAKE * dt)

        if self._ch_rpm <= 0.01:
            self._ch_rpm = 0.0
            self._ch_load = 0.0
            self._ch_current = 0.0
            self._ch_torque = 0.0
            return

        # Carga = f(rpm, dureza del terreno, empuje aplicado)
        load = 8.0 + 95.0 * (self._ch_rpm / CUTTERHEAD_RPM_MAX) * self._ground_hardness
        if self._cmd["advance_enabled"] and self._prop_speed > 1.0:
            load *= 1.0 + 0.55 * (self._prop_speed / PROPULSION_SPEED_MAX)
        if "cutterhead_overload" in self._faults:
            load += CUTTERHEAD_FAULT_LOAD_PCT
        load += self._rng.uniform(-1.5, 1.5)

        self._ch_load = max(0.0, load)
        self._ch_current = max(
            0.0, self._ch_load * CUTTERHEAD_AMPS_PER_PCT + self._rng.uniform(-0.25, 0.25)
        )
        self._ch_torque = CUTTERHEAD_TORQUE_MAX_NM * self._ch_load / 100.0

    def _update_bladder(self, dt):
        cmd = self._cmd
        inflate_ok = cmd["bladder_inflate_enable"]
        pressure = self._bladder_p

        if "bladder_overpressure" in self._faults:
            # Valvula de inflado trabada: sube ignorando la consigna.
            pressure += BLADDER_PUMP_STUCK_RATE * dt
        elif inflate_ok and cmd["bladder_pressure_setpoint_bar"] > 0.0:
            error = cmd["bladder_pressure_setpoint_bar"] - pressure
            if error > 0.0:
                pressure += min(BLADDER_INFLATE_RATE, error * 2.5) * dt
            elif error < -0.05:
                pressure -= min(BLADDER_INFLATE_RATE, -error * 2.0) * dt

        pressure -= BLADDER_LEAK_RATE * dt
        if "bladder_underpressure" in self._faults:
            pressure -= BLADDER_LEAK_FAULT_RATE * dt
        if cmd["relief_valve_open"]:
            pressure -= BLADDER_RELIEF_RATE * dt
        if not cmd["main_power"] and "bladder_overpressure" not in self._faults:
            pressure -= BLADDER_UNPOWERED_LEAK_RATE * dt   # sin energia se despresuriza

        self._bladder_p = max(0.0, pressure)

    def _update_propulsion(self, dt):
        if self._cmd["safe_torque_off"]:
            self._zero_drives()
        else:
            target = self._cmd["propulsion_speed_setpoint_mm_min"]
            if self._prop_speed < target:
                self._prop_speed = min(target, self._prop_speed + PROPULSION_RAMP * dt)
            else:
                self._prop_speed = max(target, self._prop_speed - PROPULSION_RAMP * dt)
            if self._prop_speed < 0.05:
                self._prop_speed = 0.0

        # Orugas de propulsion: el motor A introduce un diferencial de velocidad.
        differential = self._steer_a / STEER_STROKE_MM
        self._track_left = self._prop_speed * (1.0 + 0.25 * differential)
        self._track_right = self._prop_speed * (1.0 - 0.25 * differential)
        self._thrust = self._bladder_p * 9.5 * (
            0.35 + 0.65 * self._prop_speed / PROPULSION_SPEED_MAX
        )

        # Carga y corriente del motor de propulsion.
        if self._prop_speed <= 0.05:
            self._prop_load = 0.0
            self._prop_current = 0.0
            return
        load = 10.0 + 70.0 * (self._prop_speed / PROPULSION_SPEED_MAX) * self._ground_hardness
        if "propulsion_overload" in self._faults:
            load += PROPULSION_FAULT_LOAD_PCT
        load += self._rng.uniform(-1.5, 1.5)
        self._prop_load = max(0.0, load)
        self._prop_current = max(
            0.0, self._prop_load * PROPULSION_AMPS_PER_PCT + self._rng.uniform(-0.25, 0.25)
        )

    def _update_steering(self, dt):
        if self._cmd["safe_torque_off"]:
            return                       # direccion congelada en su posicion
        step = STEER_RATE_MM_S * dt
        self._steer_a += clamp(self._cmd["steer_a_setpoint_mm"] - self._steer_a, -step, step)
        self._steer_b += clamp(self._cmd["steer_b_setpoint_mm"] - self._steer_b, -step, step)

    def _update_hydroshield(self, dt):
        target = self._cmd["hydroshield_setpoint_bar"]
        self._hydro_p += (target - self._hydro_p) * min(1.0, dt / 2.5)
        if target > 0.0:
            # Rizado: 0.02 bar por paso de 100 ms, independiente del dt real.
            self._hydro_p += 0.2 * dt * math.sin(self._sim_time * 1.3)
        self._hydro_p = max(0.0, self._hydro_p)

    def _update_navigation(self, dt):
        """Integra los 6 grados de libertad. Solo avanza con autorizacion de avance."""
        # Vibracion mecanica proporcional a las rpm del cabezal.
        self._vib_phase += dt * 2.0 * math.pi * 1.7
        amplitude = 0.35 * (self._ch_rpm / CUTTERHEAD_RPM_MAX)
        self._vib_roll = amplitude * math.sin(self._vib_phase)
        self._vib_pitch = amplitude * 0.5 * math.sin(self._vib_phase * 0.7)

        if not self._cmd["advance_enabled"] or self._prop_speed <= 0.0:
            self._yaw_rate = 0.0
            return

        ds = (self._prop_speed / 60000.0) * dt           # mm/min -> m en este paso
        if ds <= 0.0:
            return

        d_heading = STEER_YAW_DEG_PER_M * self._steer_a * ds
        self._heading = (self._heading + d_heading) % 360.0
        self._yaw_rate = d_heading / dt if dt > 0 else 0.0
        self._pitch = clamp(
            self._pitch + STEER_PITCH_DEG_PER_M * self._steer_b * ds,
            -PITCH_LIMIT_DEG,
            PITCH_LIMIT_DEG,
        )
        self._roll += (0.02 * self._steer_a - self._roll * 0.5) * ds

        heading_rad = math.radians(self._heading)
        pitch_rad = math.radians(self._pitch)
        horizontal = ds * math.cos(pitch_rad)

        self._chainage += ds
        self._easting += horizontal * math.sin(heading_rad)
        self._northing += horizontal * math.cos(heading_rad)
        self._elevation += ds * math.sin(pitch_rad)

    def _update_thermal(self, dt):
        tau = 60.0
        head_target = AMBIENT_TEMP_C + 0.50 * self._ch_load
        prop_target = AMBIENT_TEMP_C + 0.35 * (self._prop_speed / PROPULSION_SPEED_MAX) * 100.0
        self._ch_temp += (head_target - self._ch_temp) * min(1.0, dt / tau)
        self._prop_temp += (prop_target - self._prop_temp) * min(1.0, dt / tau)

    def _update_wireless_sensor(self, dt):
        """Bateria y latencia del sensor inalambrico de presion de vejiga."""
        drain = (BLADDER_SENSOR_DRAIN_PCT_PER_H / 3600.0) * dt
        if "bladder_sensor_battery" in self._faults:
            drain += BLADDER_SENSOR_DRAIN_FAULT_PCT_S * dt
        self._bs_battery = clamp(self._bs_battery - drain, 0.0, 100.0)

        if "bladder_sensor_signal" in self._faults or self._bs_battery <= 0.0:
            self._bs_link_ok = False
            self._bs_signal_ms = BLADDER_SENSOR_SIGNAL_TIMEOUT_MS
            return
        self._bs_link_ok = True
        wander = self._rng.gauss(0.0, NOISE_SIGNAL_MS)
        self._bs_signal_ms = max(4.0, BLADDER_SENSOR_SIGNAL_BASE_MS + wander)


# =========================================================================
# CAPA 3 - NUCLEO / DOMINIO (TBMController y logica de seguridad)
# =========================================================================
# El nucleo solo conoce `IHardwareAdapter`. Aqui viven las reglas de TBC y
# del PDB: no hay ningun import de adaptadores ni de aiohttp en esta capa.

# --- Cabezal de corte ---
CUTTERHEAD_RPM_DEFAULT = 3.5
CUTTERHEAD_RPM_MIN_MINING = 1.0     # rpm minimas para autorizar MINING
CUTTERHEAD_LOAD_MAX_PCT = 100.0     # umbral de sobrecarga
CUTTERHEAD_CURRENT_MAX_A = 32.0     # umbral de sobrecorriente
OVERLOAD_DEBOUNCE_S = 1.5           # antirebote de los interlocks de sobrecarga

# --- Propulsion ---
PROPULSION_SPEED_DEFAULT = 42.0     # mm/min
PROPULSION_LOAD_MAX_PCT = 100.0
PROPULSION_CURRENT_MAX_A = 24.0

# --- Vejiga de propulsion ---
BLADDER_P_MIN = 1.20                # bar - por debajo se inhibe la propulsion
BLADDER_P_MAX = 3.00                # bar - por encima: alivio + inhibicion
BLADDER_P_TARGET_DEFAULT = 1.80
BLADDER_P_TARGET_MAX = 2.60
BLADDER_RELIEF_HYST = 0.25          # bar de histeresis para cerrar el alivio
BLADDER_GRIP_GRACE_S = 6.0          # margen de inflado antes de armar el interlock de baja presion

# --- Hydroshield (presion de soporte del frente) ---
HYDROSHIELD_IDLE_BAR = 1.90
HYDROSHIELD_MINING_BAR = 2.45

# --- Sensor inalambrico de vejiga ---
BLADDER_SENSOR_BATTERY_CRITICAL_PCT = 10.0
BLADDER_SENSOR_SIGNAL_LOST_MS = 250.0       # umbral de perdida de enlace

# --- Comunicaciones y watchdog ---
WATCHDOG_TIMEOUT_S = 10.0           # un enlace armado sin vida > 10 s => FAULT
UPLINK_MAX_FAILURES = 3             # POST a TBC fallidos consecutivos = enlace caido
MAX_STEP_DT_S = 0.5                 # tope de dt entre ciclos de control

LINK_HARDWARE = "hardware_network"  # check_network_health() del adaptador
LINK_OPERATOR = "operator_station"  # latido de la estacion del operador
LINK_UPLINK = "tbc_uplink"          # POST de telemetria hacia TBC

# --- Subsistemas inhabilitables por interlock ---
SUB_CUTTERHEAD = "cutterhead"
SUB_PROPULSION = "propulsion"
SUB_INFLATION = "inflation"
SUB_STEERING = "steering"
ALL_SUBSYSTEMS = (SUB_CUTTERHEAD, SUB_PROPULSION, SUB_INFLATION, SUB_STEERING)


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

# Safe Restart (TBC 10.g): volver a un estado operativo desde una parada de
# emergencia, una perdida de energia o una falla exige DOS acciones
# independientes y en orden.
RESTART_SEQUENCES = {
    State.POWER_OFF: ("power_on", "arm"),
    State.E_STOPPED: ("reset_estop", "arm"),
    State.FAULT: ("clear_fault", "arm"),
}

# Nombre de la orden de parada de emergencia externa (HMI / PLC). NO es un
# comando de `POST /command`: entra por `TBMController.engage_estop()` y no
# pasa por el despacho de comandos ni por la validacion de transiciones.
ESTOP_COMMAND = "engage_estop"

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

    def trip(self, wall_time):
        """Engancha el enclavamiento. Devuelve True solo la primera vez."""
        self.active = True
        if self.latched:
            return False
        self.latched = True
        self.triggered_at = wall_time
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


class LinkWatchdog:
    """Watchdog de enlaces: mide hace cuanto cada enlace no da señales de vida."""

    def __init__(self, timeout_s, now):
        self.timeout_s = float(timeout_s)
        self._born = now
        self._last_alive = {}

    def alive(self, link, at):
        previous = self._last_alive.get(link)
        if previous is None or at > previous:
            self._last_alive[link] = at

    def silence(self, link, now):
        return max(0.0, now - self._last_alive.get(link, self._born))

    def expired(self, links, now):
        """[(enlace, segundos_en_silencio)] de los enlaces que superan el timeout."""
        result = []
        for link in links:
            silence = self.silence(link, now)
            if silence > self.timeout_s:
                result.append((link, silence))
        return result

    def worst_silence(self, links, now):
        return max((self.silence(link, now) for link in links), default=0.0)


class CommandOutcome(Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"           # valido pero no permitido ahora (interlock, estado...)
    INVALID = "invalid"             # comando desconocido o parametros invalidos


@dataclass(frozen=True)
class CommandResult:
    """Resultado de un comando de operador (el nucleo no conoce HTTP)."""

    ok: bool
    outcome: CommandOutcome
    command: str
    message: str
    state: State
    mining: bool
    estop_engaged: bool
    active_interlocks: list
    latched_interlocks: list
    safe_restart: dict
    timestamp: int


@dataclass(frozen=True)
class EstopResult:
    """Resultado de una parada de emergencia externa (HMI / PLC).

    El nucleo no conoce HTTP ni el texto que vera el HMI: solo informa que
    paso. `actuators_zeroed` es una MEDICION (el hardware acepto las
    consignas a 0 y el STO), no una suposicion.
    """

    command: str
    source: str
    state: State
    previous_state: State
    estop_engaged: bool
    actuators_zeroed: bool
    already_stopped: bool
    timestamp: int


@dataclass(frozen=True)
class ControllerSnapshot:
    """Foto inmutable del controlador (un ciclo de 10 Hz). Es la unica fuente
    de datos de la capa de presentacion."""

    wall_time: float
    sample_index: int
    uptime_s: float
    adapter: str
    state: State
    previous_state: State
    state_elapsed_s: float
    estop_engaged: bool
    hardware_estop: bool
    sensors: dict
    actuator_commands: dict
    inhibited: dict
    cutterhead_running: bool
    cutterhead_rpm_setpoint: float
    propulsion_enabled: bool
    propulsion_speed_setpoint: float
    bladder_setpoint_bar: float
    bladder_inflating: bool
    relief_valve_open: bool
    steer_a_setpoint_mm: float
    steer_b_setpoint_mm: float
    mined_length_m: float
    mining_time_s: float
    interlocks: dict
    active_interlocks: list
    latched_interlocks: list
    safe_restart: dict
    allowed_transitions: list
    comms: dict
    commands_received: int
    last_command: Optional[str]
    injected_faults: list


class TBMController:
    """Cerebro supervisor de la tuneladora (nucleo / dominio).

    Recibe un `IHardwareAdapter` por inyeccion de dependencias y ejecuta un
    lazo de control NO bloqueante a 10 Hz (`run()` / `step()`):

        leer sensores -> supervisar (E-Stop, interlocks, watchdog)
                      -> calcular consignas con compuerta de seguridad
                      -> escribir actuadores -> publicar snapshot

    Todo corre en el hilo del event loop, por lo que el estado no necesita
    candados: los comandos de la API y el lazo de control nunca se solapan.
    """

    def __init__(
        self,
        hardware,
        *,
        watchdog_timeout_s=WATCHDOG_TIMEOUT_S,
        start_state=State.POWER_OFF,
        clock=time.monotonic,
        wall_clock=time.time,
    ):
        if not isinstance(hardware, IHardwareAdapter):
            raise TypeError("The hardware must implement IHardwareAdapter.")
        self._hw = hardware
        self._clock = clock
        self._wall = wall_clock
        self.watchdog_timeout_s = float(watchdog_timeout_s)

        boot = clock()
        self._boot = boot

        # --- Maquina de estados ---
        self.state = State.POWER_OFF
        self.previous_state = State.POWER_OFF
        self._state_since = boot
        self.estop_engaged = False
        self._restart_done = set()

        # --- Intencion del operador (se filtra por la compuerta de seguridad) ---
        self.cutterhead_running = False
        self.cutterhead_rpm_target = 0.0
        self.bladder_inflating = False
        self.bladder_target = 0.0
        self.relief_valve_open = False
        self.propulsion_enabled = False
        self.propulsion_speed_target = PROPULSION_SPEED_DEFAULT
        self.steer_a_target_mm = 0.0
        self.steer_b_target_mm = 0.0

        # --- Enclavamientos ---
        self.interlocks = {
            "cutterhead_overload": Interlock(
                "cutterhead_overload",
                "Overcurrent or overload on the cutterhead",
                {SUB_CUTTERHEAD, SUB_PROPULSION},
                forces_fault=True,
            ),
            "propulsion_overload": Interlock(
                "propulsion_overload",
                "Overcurrent or overload on the propulsion motor",
                {SUB_PROPULSION},
                forces_fault=False,
            ),
            "bladder_underpressure": Interlock(
                "bladder_underpressure",
                "Propulsion bladder pressure below P_min",
                {SUB_PROPULSION},
                forces_fault=False,
            ),
            "bladder_overpressure": Interlock(
                "bladder_overpressure",
                "Propulsion bladder overpressure (P > P_max)",
                {SUB_PROPULSION, SUB_INFLATION},
                forces_fault=True,
            ),
            "comms_loss": Interlock(
                "comms_loss",
                "Network communication loss: global abort",
                {SUB_CUTTERHEAD, SUB_PROPULSION, SUB_INFLATION, SUB_STEERING},
                forces_fault=True,
            ),
            "bladder_sensor": Interlock(
                "bladder_sensor",
                "Wireless bladder sensor: link lost or critical battery",
                {SUB_CUTTERHEAD, SUB_PROPULSION, SUB_INFLATION, SUB_STEERING},
                forces_fault=True,
            ),
        }

        # --- Lecturas y E/S con el hardware ---
        self._sensors = {}
        self._start_chainage = None
        self._last_commands = safe_actuator_commands()
        self._io_ok = True
        self._io_prev_ok = True
        self._io_error_logged = False
        self._hw_link_ok = True
        self._hw_estop_now = False
        self._hw_estop_prev = False

        # --- Watchdog de enlaces ---
        self._watchdog = LinkWatchdog(self.watchdog_timeout_s, boot)
        self._last_operator_contact = boot
        self.uplink_ok = None
        self.uplink_failures = 0
        self.uplink_sent = 0

        # --- Temporizadores, contadores y publicacion ---
        self._last_step = None
        self._overload_timer = 0.0
        self._prop_overload_timer = 0.0
        self._grip_timer = 0.0
        self.mining_seconds = 0.0
        self.tick_count = 0
        self.commands_received = 0
        self.last_command = None
        self._snapshot = None

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
            "set_bladder_sensor": self._cmd_set_bladder_sensor,
            "hardware_estop": self._cmd_hardware_estop,
        }

        if start_state is not State.POWER_OFF:
            self.state = start_state
            self.previous_state = State.POWER_OFF
            self._on_enter_state(start_state, State.POWER_OFF)
            log_warn(
                f"Initial state forced to {start_state.value} via command line "
                "(development mode, power-on sequence skipped)."
            )

    # -------------------------------------------------------------------
    # 3.1 API publica del nucleo
    # -------------------------------------------------------------------

    @property
    def adapter_name(self):
        return type(self._hw).__name__

    def command_names(self):
        return sorted(self._commands)

    async def run(self, period_s=TICK_PERIOD_S):
        """Lazo de control no bloqueante (tarea asyncio) a 1/period_s Hz."""
        log_info(
            f"Core control loop active at {1.0 / period_s:.0f} Hz "
            f"(adapter: {self.adapter_name})."
        )
        next_tick = self._clock()
        failures = 0
        while True:
            try:
                self.step()
                failures = 0
            except Exception as exc:               # el lazo nunca debe morir
                failures += 1
                if failures == 1 or failures % 50 == 0:
                    log_warn(f"Control loop error: {exc!r}")
                self._fail_safe()
            next_tick += period_s
            delay = next_tick - self._clock()
            if delay <= 0.0:
                next_tick = self._clock()
                delay = 0.0
            await asyncio.sleep(delay)

    def step(self):
        """Un ciclo de control. SIEMPRE lee los sensores, en cualquier estado
        (incluido E-STOPPED), para mantener viva la telemetria de baja tension."""
        now = self._clock()
        if self._last_step is None:
            dt = TICK_PERIOD_S
        else:
            dt = clamp(now - self._last_step, 0.0, MAX_STEP_DT_S)
        self._last_step = now

        self._acquire(now)
        self._supervise(now, dt)
        self._drive_outputs()
        self.tick_count += 1
        self._publish()

    def snapshot(self):
        """Ultima foto publicada a 10 Hz (inmutable)."""
        if self._snapshot is None:
            self.step()
        return self._snapshot

    def touch_operator(self):
        """La estacion del operador dio señales de vida (latido del watchdog)."""
        self._last_operator_contact = self._clock()

    def register_uplink_result(self, ok, detail=""):
        """El cliente TBC informa el resultado de cada POST."""
        if ok:
            if self.uplink_failures > 0:
                log_net("Telemetry link to TBC restored.")
            self.uplink_ok = True
            self.uplink_failures = 0
            self.uplink_sent += 1
            return
        self.uplink_ok = False
        self.uplink_failures += 1
        if self.uplink_failures == 1:
            log_net(f"Telemetry upload to TBC failed ({detail}).")
        if self.uplink_failures == UPLINK_MAX_FAILURES:
            log_net(
                f"{self.uplink_failures} consecutive telemetry uploads failed "
                f"({detail}): TBC link down; the "
                f"{self.watchdog_timeout_s:.0f} s watchdog is counting down."
            )

    def execute(self, command, params):
        """Ejecuta un comando de operador. Nunca se salta un interlock."""
        self.touch_operator()
        name = str(command).strip().lower()
        params = params if isinstance(params, dict) else {}

        handler = self._commands.get(name)
        if handler is None:
            log_warn(f"Unknown command: '{name}'")
            return self._result(
                False, CommandOutcome.INVALID, name,
                f"Unknown command '{name}'. Valid commands: "
                f"{', '.join(self.command_names())}.",
            )

        self.commands_received += 1
        self.last_command = name
        log_command(f"'{name}' params={json.dumps(params, ensure_ascii=False)} "
                    f"[state={self.state.value}]")
        try:
            ok, message = handler(params)
        except (TypeError, ValueError) as exc:
            log_warn(f"'{name}' with invalid parameters: {exc}")
            self._flush_outputs()
            return self._result(
                False, CommandOutcome.INVALID, name, f"Invalid parameters: {exc}"
            )

        if ok:
            log_info(f"'{name}' ACCEPTED -> {message}")
        else:
            log_warn(f"'{name}' REJECTED -> {message}")
        self._flush_outputs()
        outcome = CommandOutcome.ACCEPTED if ok else CommandOutcome.REJECTED
        return self._result(ok, outcome, name, message)

    def engage_estop(self, source="UNKNOWN"):
        """Parada de emergencia externa (HMI / PLC): fuerza E-STOPPED YA.

        A diferencia de `execute()`, NO pasa por el despacho de comandos ni
        por `_request_state()`/`_preconditions()`: ninguna validacion de la
        maquina de estados ni ningun interlock puede rechazarla, y funciona
        desde cualquier estado (incluidos POWER OFF y FAULT). Es idempotente.

        Es SINCRONA y no hace `await`: se completa de forma atomica entre dos
        ciclos del lazo de `run()`, que ni se detiene ni se bloquea. Tras la
        parada el lazo sigue llamando a `read_sensors()` a 10 Hz, asi que la
        telemetria 6DOF y las presiones siguen fluyendo.

        No toca los interlocks (enganchados o activos siguen igual) ni relaja
        el Safe Restart: salir de E-STOPPED exige 'reset_estop' + 'arm', y una
        parada repetida invalida cualquier reinicio a medias.
        """
        previous = self.state
        self.commands_received += 1
        self.last_command = ESTOP_COMMAND
        log_command(f"'{ESTOP_COMMAND}' source={source} [state={previous.value}]")

        already = self._engage_estop(f"EXTERNAL EMERGENCY STOP ({source})")
        self._flush_outputs()               # ceros al hardware ANTES de confirmar

        commands = self._last_commands
        zeroed = bool(
            commands["safe_torque_off"]
            and commands["cutterhead_rpm_setpoint"] == 0.0
            and commands["propulsion_speed_setpoint_mm_min"] == 0.0
        )
        if not zeroed:
            log_warn(
                f"'{ESTOP_COMMAND}': hardware did not confirm setpoints at 0/STO; "
                "the control loop retries the write on every cycle."
            )
        return EstopResult(
            command=ESTOP_COMMAND,
            source=source,
            state=self.state,
            previous_state=previous,
            estop_engaged=self.estop_engaged,
            actuators_zeroed=zeroed,
            already_stopped=already,
            timestamp=int(self._wall()),
        )

    def _result(self, ok, outcome, command, message):
        return CommandResult(
            ok=ok,
            outcome=outcome,
            command=command,
            message=message,
            state=self.state,
            mining=self.state is State.MINING,
            estop_engaged=self.estop_engaged,
            active_interlocks=sorted(k for k, lock in self.interlocks.items() if lock.active),
            latched_interlocks=self._latched_keys(),
            safe_restart=self._restart_status(),
            timestamp=int(self._wall()),
        )

    # -------------------------------------------------------------------
    # 3.2 Adquisicion de datos (puerto de hardware)
    # -------------------------------------------------------------------

    def _acquire(self, now):
        """Lee sensores, salud de red y hongo fisico. Ante cualquier error de
        E/S falla hacia el lado seguro."""
        ok = True

        try:
            raw = self._hw.read_sensors()
            missing = [key for key in SENSOR_KEYS if key not in raw]
            if missing:
                raise HardwareError(f"read_sensors() missing keys {missing}")
            self._sensors = dict(raw)
            if self._start_chainage is None:
                self._start_chainage = self._sensors["chainage_odometry_m"]
        except Exception as exc:
            ok = False
            self._report_io_error("read_sensors", exc)

        network_ok = self._hardware_network_ok()
        if network_ok is None:
            ok = False
            network_ok = False
        self._hw_link_ok = bool(network_ok) and ok
        if self._hw_link_ok:
            self._watchdog.alive(LINK_HARDWARE, now)

        self._hw_estop_now = self._hardware_estop_pressed()
        self._io_ok = ok
        if ok:
            self._io_error_logged = False

    def _hardware_network_ok(self):
        """check_network_health() protegido. None = error de E/S."""
        try:
            return bool(self._hw.check_network_health())
        except Exception as exc:
            self._report_io_error("check_network_health", exc)
            return None

    def _hardware_estop_pressed(self):
        """Hongo fisico. Una falla al leerlo se trata como PRESIONADO (fail-safe)."""
        try:
            return bool(self._hw.is_hardware_estop_triggered())
        except Exception as exc:
            self._report_io_error("is_hardware_estop_triggered", exc)
            return True

    def _report_io_error(self, operation, exc):
        if not self._io_error_logged:
            self._io_error_logged = True
            log_interlock(f"Hardware I/O error in {operation}(): {exc!r}")

    def _sensed(self, key, default=0.0):
        return self._sensors.get(key, default)

    # -------------------------------------------------------------------
    # 3.3 Supervision: E-Stop, temporizadores, interlocks y watchdog
    # -------------------------------------------------------------------

    def _supervise(self, now, dt):
        if not self._io_ok and self._io_prev_ok:
            log_interlock(
                "Hardware I/O failure: setpoints set to safe state (STO) "
                "and actuation stopped."
            )
            self._stop_all_actuation()
            if self.state is State.MINING:
                self._set_state(State.READY, "mining aborted due to hardware I/O failure")
        self._io_prev_ok = self._io_ok

        self._handle_hardware_estop()

        if self.state is State.MINING:
            self.mining_seconds += dt

        inflate_ok = self.bladder_inflating and not self._inhibited(SUB_INFLATION)
        gripping = inflate_ok and self.bladder_target >= BLADDER_P_MIN
        self._grip_timer = self._grip_timer + dt if gripping else 0.0

        if self._start_chainage is not None:
            self._check_interlocks(now, dt)

        if self.state in (State.E_STOPPED, State.FAULT):
            self._zero_actuators()

    def _handle_hardware_estop(self):
        pressed = self._hw_estop_now
        if pressed and not self._hw_estop_prev:
            log_command("PHYSICAL EMERGENCY STOP detected on the hardware.")
            self._restart_done = set()          # el reinicio en curso se invalida
        self._hw_estop_prev = pressed
        if not pressed:
            return
        self.estop_engaged = True
        if self.state is not State.E_STOPPED:
            self._set_state(State.E_STOPPED, "physical E-stop button (hardware)")

    def _armed_links(self):
        links = []
        if self.state is not State.POWER_OFF:
            links += [LINK_HARDWARE, LINK_UPLINK]
        if self.state in (State.READY, State.MINING):
            links.append(LINK_OPERATOR)
        return links

    def _check_interlocks(self, now, dt):
        s = self._sensors

        # --- 1. Sobrecorriente / sobrecarga del cabezal de corte ---
        lock = self.interlocks["cutterhead_overload"]
        overload = s["cutterhead_rpm"] > 0.0 and (
            s["cutterhead_load_pct"] > CUTTERHEAD_LOAD_MAX_PCT
            or s["cutterhead_current_a"] > CUTTERHEAD_CURRENT_MAX_A
        )
        lock.active = overload
        self._overload_timer = (
            self._overload_timer + dt if overload else max(0.0, self._overload_timer - dt)
        )
        if self._overload_timer >= OVERLOAD_DEBOUNCE_S:
            self._trip(
                lock,
                f"load {s['cutterhead_load_pct']:.0f} % / {s['cutterhead_current_a']:.1f} A "
                f"(limits {CUTTERHEAD_LOAD_MAX_PCT:.0f} % / {CUTTERHEAD_CURRENT_MAX_A:.1f} A)",
            )

        # --- 2. Sobrecorriente / sobrecarga del motor de propulsion ---
        lock = self.interlocks["propulsion_overload"]
        prop_overload = s["propulsion_speed_mm_min"] > 0.0 and (
            s["propulsion_load_pct"] > PROPULSION_LOAD_MAX_PCT
            or s["propulsion_current_a"] > PROPULSION_CURRENT_MAX_A
        )
        lock.active = prop_overload
        self._prop_overload_timer = (
            self._prop_overload_timer + dt
            if prop_overload else max(0.0, self._prop_overload_timer - dt)
        )
        if self._prop_overload_timer >= OVERLOAD_DEBOUNCE_S:
            self._trip(
                lock,
                f"load {s['propulsion_load_pct']:.0f} % / {s['propulsion_current_a']:.1f} A "
                f"(limits {PROPULSION_LOAD_MAX_PCT:.0f} % / {PROPULSION_CURRENT_MAX_A:.1f} A)",
            )

        # --- 3. Perdida de presion en la vejiga de propulsion ---
        pressure = s["bladder_pressure_bar"]
        lock = self.interlocks["bladder_underpressure"]
        armed = (
            self.state in (State.READY, State.MINING)
            and self.bladder_target >= BLADDER_P_MIN
            and self._grip_timer >= BLADDER_GRIP_GRACE_S
        )
        lock.active = armed and pressure < BLADDER_P_MIN
        if lock.active:
            self._trip(
                lock,
                f"P_bladder {pressure:.2f} bar < P_min {BLADDER_P_MIN:.2f} bar",
            )

        # --- 4. Sobrepresion en la vejiga: detiene inflado y dispara el alivio ---
        lock = self.interlocks["bladder_overpressure"]
        lock.active = pressure > BLADDER_P_MAX
        if lock.active:
            if not self.relief_valve_open:
                self.relief_valve_open = True
                log_interlock("Pressure relief valve OPENED.")
            self._trip(
                lock,
                f"P_bladder {pressure:.2f} bar > P_max {BLADDER_P_MAX:.2f} bar",
            )
        elif self.relief_valve_open and pressure <= BLADDER_P_MAX - BLADDER_RELIEF_HYST:
            self.relief_valve_open = False
            log_interlock(f"Relief valve CLOSED (P_bladder {pressure:.2f} bar).")

        # --- 5. Watchdog de red: aborto global si un enlace armado calla > 10 s ---
        lock = self.interlocks["comms_loss"]
        if self.uplink_failures < UPLINK_MAX_FAILURES:
            self._watchdog.alive(LINK_UPLINK, now)
        self._watchdog.alive(LINK_OPERATOR, self._last_operator_contact)
        expired = self._watchdog.expired(self._armed_links(), now)
        lock.active = bool(expired)
        if expired:
            detail = "; ".join(
                f"link '{link}' with no sign of life for {silence:.1f} s"
                for link, silence in expired
            ) + f" (watchdog {self.watchdog_timeout_s:.1f} s)"
            self._trip(lock, detail)

        # --- 6. Sensor inalambrico de vejiga: bateria critica o enlace perdido ---
        lock = self.interlocks["bladder_sensor"]
        battery = s["bladder_sensor_battery_pct"]
        signal_ms = s["bladder_sensor_signal_ms"]
        battery_critical = battery <= BLADDER_SENSOR_BATTERY_CRITICAL_PCT
        signal_lost = (
            not s["bladder_sensor_link_ok"] or signal_ms >= BLADDER_SENSOR_SIGNAL_LOST_MS
        )
        lock.active = battery_critical or signal_lost
        if lock.active:
            if battery_critical and signal_lost:
                detail = f"battery {battery:.1f} % and signal {signal_ms:.0f} ms"
            elif battery_critical:
                detail = (
                    f"battery {battery:.1f} % <= {BLADDER_SENSOR_BATTERY_CRITICAL_PCT:.0f} %"
                )
            else:
                detail = (
                    f"signal {signal_ms:.0f} ms >= {BLADDER_SENSOR_SIGNAL_LOST_MS:.0f} ms "
                    "(link lost)"
                )
            self._trip(lock, detail)

    def _trip(self, lock, detail):
        """Engancha un interlock y aplica sus acciones de mitigacion."""
        if not lock.trip(self._wall()):
            return

        log_interlock(f"TRIPPED '{lock.key}': {lock.description}.")
        log_interlock(f"   Cause: {detail}")
        log_interlock(f"   Inhibits: {', '.join(sorted(lock.inhibits))}")

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
            self._set_state(State.READY, f"mining aborted by interlock '{lock.key}'")

    def _condition_present(self, key):
        """Reevalua en el instante si la causa fisica de un interlock persiste."""
        s = self._sensors
        if key == "cutterhead_overload":
            return s["cutterhead_rpm"] > 0.0 and (
                s["cutterhead_load_pct"] > CUTTERHEAD_LOAD_MAX_PCT
                or s["cutterhead_current_a"] > CUTTERHEAD_CURRENT_MAX_A
            )
        if key == "propulsion_overload":
            return s["propulsion_speed_mm_min"] > 0.0 and (
                s["propulsion_load_pct"] > PROPULSION_LOAD_MAX_PCT
                or s["propulsion_current_a"] > PROPULSION_CURRENT_MAX_A
            )
        if key == "bladder_underpressure":
            return (
                self.bladder_target >= BLADDER_P_MIN
                and s["bladder_pressure_bar"] < BLADDER_P_MIN
                and self.state in (State.READY, State.MINING)
            )
        if key == "bladder_overpressure":
            return s["bladder_pressure_bar"] > BLADDER_P_MAX
        if key == "comms_loss":
            return (not self._hw_link_ok) or self.uplink_failures >= UPLINK_MAX_FAILURES
        if key == "bladder_sensor":
            return (
                not s["bladder_sensor_link_ok"]
                or s["bladder_sensor_battery_pct"] <= BLADDER_SENSOR_BATTERY_CRITICAL_PCT
                or s["bladder_sensor_signal_ms"] >= BLADDER_SENSOR_SIGNAL_LOST_MS
            )
        return False

    # -------------------------------------------------------------------
    # 3.4 Salidas: consignas con compuerta de seguridad
    # -------------------------------------------------------------------

    def _inhibited(self, subsystem):
        """True si el subsistema esta inhabilitado por estado, E-Stop o interlock."""
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return True
        if self.estop_engaged or self._hw_estop_now:
            return True
        return any(
            lock.latched and subsystem in lock.inhibits
            for lock in self.interlocks.values()
        )

    def _latched_keys(self):
        return sorted(key for key, lock in self.interlocks.items() if lock.latched)

    def _compute_commands(self):
        """Traduce la intencion del operador en consignas de hardware.

        COMPUERTA DE SEGURIDAD (PDB 6.6.2): es el unico camino hacia los
        actuadores y se aplica SIEMPRE, sin importar como se emitio la
        intencion (comando manual, estado, etc.): un comando manual nunca
        puede saltarse un interlock.
        """
        state = self.state
        sto = (
            state in (State.E_STOPPED, State.FAULT)
            or self.estop_engaged
            or self._hw_estop_now
            or not self._io_ok
        )

        cutter_ok = self.cutterhead_running and not self._inhibited(SUB_CUTTERHEAD) and not sto
        inflate_ok = self.bladder_inflating and not self._inhibited(SUB_INFLATION) and not sto

        pressure = self._sensed("bladder_pressure_bar")
        propulsion_ok = (
            state is State.MINING
            and not sto
            and not self._inhibited(SUB_PROPULSION)
            and pressure >= BLADDER_P_MIN
        )
        self.propulsion_enabled = propulsion_ok

        if self._inhibited(SUB_STEERING) or sto:
            # Direccion inhabilitada: mantener la posicion actual.
            self.steer_a_target_mm = self._sensed("steer_a_mm")
            self.steer_b_target_mm = self._sensed("steer_b_mm")

        if state is State.POWER_OFF:
            hydro = 0.0
        elif state is State.MINING:
            hydro = HYDROSHIELD_MINING_BAR
        else:
            hydro = HYDROSHIELD_IDLE_BAR

        return {
            "safe_torque_off": sto,
            "main_power": state is not State.POWER_OFF,
            "advance_enabled": state is State.MINING and not sto,
            "cutterhead_rpm_setpoint": self.cutterhead_rpm_target if cutter_ok else 0.0,
            "propulsion_speed_setpoint_mm_min": (
                self.propulsion_speed_target if propulsion_ok else 0.0
            ),
            "bladder_inflate_enable": inflate_ok,
            "bladder_pressure_setpoint_bar": self.bladder_target,
            "relief_valve_open": self.relief_valve_open,
            "steer_a_setpoint_mm": self.steer_a_target_mm,
            "steer_b_setpoint_mm": self.steer_b_target_mm,
            "hydroshield_setpoint_bar": hydro,
        }

    def _drive_outputs(self):
        commands = self._compute_commands()
        try:
            self._hw.write_actuators(commands)
            self._last_commands = commands
        except Exception as exc:
            self._io_ok = False
            self._report_io_error("write_actuators", exc)

    def _flush_outputs(self):
        """Tras un comando: consignas al hardware YA (sin esperar al ciclo de
        100 ms), relectura y nueva foto."""
        now = self._clock()
        self._drive_outputs()
        self._acquire(now)
        self._publish()

    def _fail_safe(self):
        try:
            self._hw.write_actuators(safe_actuator_commands())
        except Exception:
            pass

    # -------------------------------------------------------------------
    # 3.5 Transiciones de estado
    # -------------------------------------------------------------------

    def _set_state(self, new_state, reason):
        if new_state is self.state:
            return
        previous = self.state
        self.previous_state = previous
        self.state = new_state
        self._state_since = self._clock()
        self._restart_done = set()
        log_state(f"{previous.value}  ->  {new_state.value}   ({reason})")
        self._on_enter_state(new_state, previous)

    def _on_enter_state(self, new_state, previous):
        """Acciones automaticas al entrar en un estado."""
        if new_state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            self._stop_all_actuation()
            if new_state in (State.E_STOPPED, State.FAULT):
                self._zero_actuators()
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
            self.steer_a_target_mm = self._sensed("steer_a_mm")
            self.steer_b_target_mm = self._sensed("steer_b_mm")

    def _stop_all_actuation(self):
        for subsystem in ALL_SUBSYSTEMS:
            self._stop_subsystem(subsystem)

    def _zero_actuators(self):
        """Intencion a 0 de inmediato (E-STOP / FAULT). El STO del hardware
        corta los accionamientos; presiones y pose 6DOF siguen vivas."""
        self.cutterhead_running = False
        self.cutterhead_rpm_target = 0.0
        self.propulsion_enabled = False
        self.propulsion_speed_target = 0.0
        self.bladder_inflating = False

    # -------------------------------------------------------------------
    # 3.6 Publicacion de la foto de estado
    # -------------------------------------------------------------------

    def _publish(self):
        now = self._clock()
        comms_lock = self.interlocks["comms_loss"]
        armed = self._armed_links()
        s = self._sensors
        mined = (
            s["chainage_odometry_m"] - self._start_chainage
            if self._start_chainage is not None and s else 0.0
        )
        injected = (
            self._hw.injected_faults() if isinstance(self._hw, IFaultInjectionPort) else []
        )
        self._snapshot = ControllerSnapshot(
            wall_time=self._wall(),
            sample_index=self.tick_count,
            uptime_s=now - self._boot,
            adapter=self.adapter_name,
            state=self.state,
            previous_state=self.previous_state,
            state_elapsed_s=now - self._state_since,
            estop_engaged=self.estop_engaged,
            hardware_estop=self._hw_estop_now,
            sensors=dict(s),
            actuator_commands=dict(self._last_commands),
            inhibited={sub: self._inhibited(sub) for sub in ALL_SUBSYSTEMS},
            cutterhead_running=self.cutterhead_running,
            cutterhead_rpm_setpoint=self.cutterhead_rpm_target,
            propulsion_enabled=self.propulsion_enabled,
            propulsion_speed_setpoint=self.propulsion_speed_target,
            bladder_setpoint_bar=self.bladder_target,
            bladder_inflating=self.bladder_inflating and not self._inhibited(SUB_INFLATION),
            relief_valve_open=self.relief_valve_open,
            steer_a_setpoint_mm=self.steer_a_target_mm,
            steer_b_setpoint_mm=self.steer_b_target_mm,
            mined_length_m=mined,
            mining_time_s=self.mining_seconds,
            interlocks={key: lock.snapshot() for key, lock in self.interlocks.items()},
            active_interlocks=sorted(k for k, lock in self.interlocks.items() if lock.active),
            latched_interlocks=self._latched_keys(),
            safe_restart=self._restart_status(),
            allowed_transitions=sorted(t.value for t in ALLOWED_TRANSITIONS[self.state]),
            comms={
                "ok": not comms_lock.latched,
                "seconds_since_last_contact": max(0.0, now - self._last_operator_contact),
                "timeout_s": self.watchdog_timeout_s,
                "uplink_ok": self.uplink_ok,
                "uplink_failures": self.uplink_failures,
                "uplink_sent": self.uplink_sent,
                "network_healthy": self._hw_link_ok,
                "io_ok": self._io_ok,
                "watchdog_armed_links": armed,
                "watchdog_silence_s": self._watchdog.worst_silence(armed, now),
            },
            commands_received=self.commands_received,
            last_command=self.last_command,
            injected_faults=injected,
        )

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

    # -------------------------------------------------------------------
    # 3.7 Comandos: energia, emergencia y Safe Restart (TBC 10.g)
    # -------------------------------------------------------------------

    def _mark_restart_step(self, step, note):
        """Registra un paso de la secuencia de reinicio seguro (2 acciones)."""
        sequence = RESTART_SEQUENCES.get(self.state)
        if sequence is None or step not in sequence:
            return False, (
                f"Step '{step}' does not apply to state {self.state.value}."
            )

        index = sequence.index(step)
        missing = [s for s in sequence[:index] if s not in self._restart_done]
        if missing:
            return False, (
                "Incomplete Safe Restart sequence: you must first run "
                f"'{missing[0]}' before '{step}'."
            )
        if step in self._restart_done:
            pending = [s for s in sequence if s not in self._restart_done]
            return True, (
                f"Step '{step}' was already registered. Remaining: {', '.join(pending)}."
            )

        is_last = index == len(sequence) - 1
        if is_last:
            blocked = self._restart_blockers()
            if blocked:
                return False, "Re-arm BLOCKED: " + "; ".join(blocked)

        self._restart_done.add(step)
        done, total = len(self._restart_done), len(sequence)
        log_info(f"Safe Restart {done}/{total}: {note}")

        if done < total:
            pending = [s for s in sequence if s not in self._restart_done]
            return True, (
                f"Action {done}/{total} registered ({note}). "
                f"{len(pending)} independent action(s) remaining: {', '.join(pending)}."
            )

        self._set_state(
            State.IDLE,
            f"Safe Restart completed with {total} independent actions",
        )
        return True, "Safe Restart complete. Machine in IDLE."

    def _restart_blockers(self):
        blockers = []
        if self._hardware_estop_pressed():
            blockers.append("the PHYSICAL E-stop button is still pressed")
        elif self.estop_engaged:
            blockers.append("the E-stop remains latched (use 'reset_estop')")
        latched = self._latched_keys()
        if latched:
            blockers.append(f"interlocks still latched: {', '.join(latched)}")
        return blockers

    def _cmd_power_on(self, params):
        if self.state is not State.POWER_OFF:
            return False, f"The machine is already powered on (state {self.state.value})."
        return self._mark_restart_step("power_on", "main power applied")

    def _cmd_power_off(self, params):
        if self.state is State.POWER_OFF:
            return True, "The machine was already powered off."
        self._set_state(State.POWER_OFF, "power cut requested by the operator")
        return True, (
            "Power cut and actuation stopped. 2 independent actions "
            "('power_on' + 'arm') are required to resume operation."
        )

    def _engage_estop(self, reason):
        """Nucleo comun de toda parada de emergencia logica (comando 'estop' y
        `engage_estop()`). Devuelve True si la maquina YA estaba en E-STOPPED.

        Fuerza el estado con `_set_state()` (sin validar transiciones). Si ya
        estaba parada, re-engancha el hongo logico, pone la intencion a 0 e
        invalida cualquier Safe Restart a medias: tras una nueva parada, las
        2 acciones independientes se cuentan desde cero.
        """
        if self.state is State.E_STOPPED:
            self.estop_engaged = True
            self._restart_done = set()
            self._zero_actuators()
            return True
        self._set_state(State.E_STOPPED, reason)
        return False

    def _cmd_estop(self, params):
        reason = str(params.get("reason", "manual operator activation"))
        if self._engage_estop(f"EMERGENCY STOP ({reason})"):
            return True, "E-STOP was already active."
        return True, (
            "E-STOP engaged: cutterhead, propulsion, inflation and steering stopped. "
            "2 independent actions ('reset_estop' + 'arm') are required to restart."
        )

    def _cmd_reset_estop(self, params):
        if self._hardware_estop_pressed():
            return False, (
                "The PHYSICAL E-stop button is still pressed: release it before "
                "running 'reset_estop'."
            )
        if self.state is not State.E_STOPPED:
            if self.estop_engaged:
                self.estop_engaged = False
                return True, "E-stop button released."
            return False, f"No active E-STOP (state {self.state.value})."
        self.estop_engaged = False
        return self._mark_restart_step("reset_estop", "E-stop button released")

    def _cmd_arm(self, params):
        if self.state not in RESTART_SEQUENCES:
            return False, (
                f"'arm' only applies in {', '.join(s.value for s in RESTART_SEQUENCES)}. "
                f"Current state: {self.state.value}."
            )
        return self._mark_restart_step("arm", "re-arm confirmed by the operator")

    def _cmd_clear_fault(self, params):
        # Retira primero las fallas inyectadas por el modo ingenieria.
        if isinstance(self._hw, IFaultInjectionPort):
            self._hw.clear_injected_faults()
        self._overload_timer = 0.0
        self._prop_overload_timer = 0.0
        self._acquire(self._clock())        # lecturas frescas para reevaluar

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
                log_interlock(f"Interlock '{key}' reset by the operator.")

        if blocking:
            return False, (
                "Cannot reset: the physical condition persists in "
                f"{', '.join(blocking)}. Fix the cause and retry."
            )

        if self.state is not State.FAULT:
            return True, (
                f"Interlocks reset ({', '.join(cleared) if cleared else 'none pending'})."
            )
        return self._mark_restart_step(
            "clear_fault",
            f"interlocks reset ({', '.join(cleared) if cleared else 'none'})",
        )

    # --- Cambio de estado general --------------------------------------

    def _cmd_set_state(self, params):
        raw = params.get("state", params.get("target"))
        if raw is None:
            return False, "Missing parameter 'state' (IDLE, READY, MINING, ...)."
        target = parse_state(raw)
        if target is None:
            return False, (
                f"Unknown state '{raw}'. Valid: "
                f"{', '.join(s.value for s in State)}."
            )
        return self._request_state(target, params)

    def _request_state(self, target, params):
        if target is self.state:
            return True, f"The machine is already in {target.value}."

        if target is State.POWER_OFF:
            return self._cmd_power_off(params)
        if target is State.E_STOPPED:
            return self._cmd_estop({"reason": "requested via set_state command"})
        if target is State.FAULT:
            return self._cmd_simulate_fault({"fault": "generic"})

        if target not in ALLOWED_TRANSITIONS[self.state]:
            allowed = ", ".join(sorted(s.value for s in ALLOWED_TRANSITIONS[self.state]))
            return False, (
                f"Unsafe transition BLOCKED: {self.state.value} -> {target.value}. "
                f"Allowed transitions from {self.state.value}: {allowed}."
            )

        if self.state in RESTART_SEQUENCES:
            sequence = " + ".join(RESTART_SEQUENCES[self.state])
            return False, (
                f"Leaving {self.state.value} requires the Safe Restart sequence "
                f"of 2 independent actions: {sequence}."
            )

        ok, why = self._preconditions(target)
        if not ok:
            return False, why

        self._set_state(target, "requested by the operator")
        return True, f"State {target.value} active."

    def _preconditions(self, target):
        latched = self._latched_keys()
        if target in (State.READY, State.MINING):
            if self.estop_engaged or self._hw_estop_now:
                return False, "The E-stop remains latched."
            if latched:
                return False, (
                    f"Latched interlocks: {', '.join(latched)}. "
                    "Run 'clear_fault' before enabling the machine."
                )
        if target is State.MINING:
            if not self.cutterhead_running or self.cutterhead_rpm_target < CUTTERHEAD_RPM_MIN_MINING:
                return False, (
                    "To mine, the cutterhead must be running with at least "
                    f"{CUTTERHEAD_RPM_MIN_MINING:.1f} rpm setpoint "
                    "(run 'start_cutterhead')."
                )
            pressure = self._sensed("bladder_pressure_bar")
            if pressure < BLADDER_P_MIN:
                return False, (
                    f"Bladder not gripping: P {pressure:.2f} bar < "
                    f"P_min {BLADDER_P_MIN:.2f} bar (run 'inflate_bladder' and wait)."
                )
        return True, ""

    # --- Cabezal de corte ----------------------------------------------

    def _cmd_start_cutterhead(self, params):
        if self.state not in (State.READY, State.MINING):
            return False, (
                "The cutterhead can only start in READY or MINING "
                f"(current state {self.state.value})."
            )
        if self._inhibited(SUB_CUTTERHEAD):
            return False, "Cutterhead inhibited by a latched interlock."
        rpm = float(params.get("rpm", self.cutterhead_rpm_target or CUTTERHEAD_RPM_DEFAULT))
        self.cutterhead_rpm_target = clamp(rpm, 0.5, CUTTERHEAD_RPM_MAX)
        self.cutterhead_running = True
        return True, f"Cutterhead running, setpoint {self.cutterhead_rpm_target:.2f} rpm."

    def _cmd_stop_cutterhead(self, params):
        was_mining = self.state is State.MINING
        self.cutterhead_running = False
        self.cutterhead_rpm_target = 0.0
        if was_mining:
            self._set_state(State.READY, "cutterhead stopped by the operator")
            return True, "Cutterhead stopped; mining suspended and machine in READY."
        return True, "Cutterhead stopped."

    def _cmd_set_cutterhead_rpm(self, params):
        if "rpm" not in params:
            return False, "Missing parameter 'rpm'."
        if self.state not in (State.READY, State.MINING):
            return False, (
                f"RPM can only be adjusted in READY or MINING (current state {self.state.value})."
            )
        rpm = clamp(float(params["rpm"]), 0.0, CUTTERHEAD_RPM_MAX)
        if rpm > 0.0 and self._inhibited(SUB_CUTTERHEAD):
            return False, "Cutterhead inhibited by a latched interlock."
        self.cutterhead_rpm_target = rpm
        if rpm <= 0.0:
            return self._cmd_stop_cutterhead({})
        self.cutterhead_running = True
        if self.state is State.MINING and rpm < CUTTERHEAD_RPM_MIN_MINING:
            self._set_state(State.READY, "cutterhead rpm below the minimum for mining")
            return True, (
                f"Setpoint {rpm:.2f} rpm below the minimum for mining; machine in READY."
            )
        return True, f"Cutterhead setpoint at {rpm:.2f} rpm."

    # --- Direccion A/B --------------------------------------------------

    def _cmd_steer(self, params):
        motor = str(params.get("motor", "A")).strip().upper()
        if motor not in ("A", "B", "BOTH"):
            return False, "Parameter 'motor' must be 'A', 'B' or 'both'."
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return False, (
                f"Steering motors do not operate in {self.state.value}."
            )
        if self._inhibited(SUB_STEERING):
            return False, "Steering inhibited by a latched interlock."

        if "position_mm" in params:
            requested = float(params["position_mm"])
            absolute = True
        elif "delta_mm" in params:
            requested = float(params["delta_mm"])
            absolute = False
        else:
            return False, "Provide 'position_mm' (absolute) or 'delta_mm' (relative)."

        targets = ("a", "b") if motor == "BOTH" else (motor.lower(),)
        applied = []
        for axis in targets:
            base = 0.0 if absolute else getattr(self, f"steer_{axis}_target_mm")
            value = clamp(base + requested, -STEER_STROKE_MM, STEER_STROKE_MM)
            setattr(self, f"steer_{axis}_target_mm", value)
            applied.append(f"{axis.upper()}={value:+.2f} mm")
        return True, (
            "Steering setpoint updated: " + ", ".join(applied)
            + f" (stroke +/-{STEER_STROKE_MM:.0f} mm)."
        )

    def _cmd_center_steering(self, params):
        if self._inhibited(SUB_STEERING):
            return False, "Steering inhibited by a latched interlock."
        self.steer_a_target_mm = 0.0
        self.steer_b_target_mm = 0.0
        return True, "Steering motors A and B centered."

    # --- Propulsion y vejiga -------------------------------------------

    def _cmd_set_propulsion_speed(self, params):
        key = "speed_mm_min" if "speed_mm_min" in params else "speed"
        if key not in params:
            return False, "Missing parameter 'speed_mm_min'."
        if self.state not in (State.READY, State.MINING):
            return False, (
                f"Propulsion can only be adjusted in READY or MINING (state {self.state.value})."
            )
        if self._inhibited(SUB_PROPULSION):
            return False, "Propulsion inhibited by a latched interlock."
        speed = clamp(float(params[key]), 0.0, PROPULSION_SPEED_MAX)
        self.propulsion_speed_target = speed
        return True, f"Propulsion setpoint at {speed:.2f} mm/min."

    def _cmd_inflate_bladder(self, params):
        if self.state in (State.POWER_OFF, State.E_STOPPED, State.FAULT):
            return False, f"Cannot inflate the bladder in {self.state.value}."
        if self._inhibited(SUB_INFLATION):
            return False, "Inflation inhibited by a latched interlock."
        target = float(params.get("target_bar", BLADDER_P_TARGET_DEFAULT))
        self.bladder_target = clamp(target, 0.0, BLADDER_P_TARGET_MAX)
        self.bladder_inflating = True
        return True, (
            f"Inflation active, setpoint {self.bladder_target:.2f} bar "
            f"(safe range {BLADDER_P_MIN:.2f}-{BLADDER_P_MAX:.2f} bar)."
        )

    def _cmd_deflate_bladder(self, params):
        was_mining = self.state is State.MINING
        self.bladder_inflating = False
        self.bladder_target = 0.0
        self._grip_timer = 0.0
        if was_mining:
            self._set_state(State.READY, "bladder depressurized by the operator")
            return True, "Bladder depressurized; mining suspended and machine in READY."
        return True, "Bladder depressurized."

    # --- Simulacion de fallas y latido ---------------------------------

    def _cmd_simulate_fault(self, params):
        raw = str(params.get("fault", params.get("type", "generic"))).strip().lower()

        if raw in ("generic", "critical"):
            if self.state in (State.POWER_OFF, State.E_STOPPED):
                return False, f"No actuation to abort in {self.state.value}."
            reason = str(params.get("reason", "critical fault simulated by the operator"))
            log_interlock(f"Generic critical fault injected: {reason}")
            self._set_state(State.FAULT, reason)
            return True, "Machine in FAULT. Use 'clear_fault' + 'arm' to recover."

        if not isinstance(self._hw, IFaultInjectionPort):
            return False, (
                f"Adapter {self.adapter_name} does not support fault injection "
                "(simulated hardware only)."
            )
        return self._hw.inject_fault(raw, params)

    def _cmd_heartbeat(self, params):
        return True, "Heartbeat received; communications watchdog refreshed."

    def _cmd_set_bladder_sensor(self, params):
        """Ajusta bateria/señal del sensor inalambrico para pruebas de filtros."""
        if not isinstance(self._hw, IFaultInjectionPort):
            return False, f"Adapter {self.adapter_name} does not support this command."
        return self._hw.set_wireless_sensor(params)

    def _cmd_hardware_estop(self, params):
        """Simula presionar/liberar el hongo de emergencia FISICO."""
        if not isinstance(self._hw, IFaultInjectionPort):
            return False, f"Adapter {self.adapter_name} does not support this command."
        pressed = bool(params.get("pressed", True))
        self._hw.set_hardware_estop(pressed)
        self._hw_estop_now = self._hardware_estop_pressed()
        self._handle_hardware_estop()
        if pressed:
            return True, (
                "PHYSICAL E-stop button pressed: immediate E-STOPPED. Release the button, then "
                "run 'reset_estop' + 'arm'."
            )
        return True, (
            "PHYSICAL E-stop button released. The machine stays in E-STOPPED until "
            "'reset_estop' + 'arm' is completed."
        )


# =========================================================================
# CAPA 4 - PRESENTACION / API (telemetria y comunicaciones de competencia)
# =========================================================================
# Unica capa que conoce HTTP, JSON y aiohttp. Solo consume la API publica
# del nucleo (`snapshot()`, `execute()`, `engage_estop()`...).

# Contrato fijo con el HMI para POST /api/plc/parada.
ESTOP_HTTP_MESSAGE = "EMERGENCY STOP ENGAGED"
ESTOP_SOURCE_DEFAULT = "UNKNOWN"
ESTOP_SOURCE_MAX_CHARS = 64

TBC_ROOT_KEYS = (
    "team", "timestamp", "mining", "chainage",
    "easting", "northing", "elevation", "roll", "pitch", "heading", "extra",
)

COMMAND_HELP = {
    "power_on": "Applies main power. Step 1/2 of the Safe Restart from POWER OFF.",
    "power_off": "Cuts power and stops all actuation. Leads to POWER OFF.",
    "estop": "Immediate emergency stop. Optional params: {'reason': text}.",
    "reset_estop": "Releases the E-stop button. Step 1/2 of the restart from E-STOPPED.",
    "clear_fault": "Resets the latched interlocks. Step 1/2 of the restart from FAULT.",
    "arm": "Confirms the re-arm. Mandatory step 2/2 of every Safe Restart; leads to IDLE.",
    "set_state": "Requests a state change. Params: {'state': 'IDLE|READY|MINING|...'}.",
    "start_cutterhead": "Starts the cutterhead. Optional params: {'rpm': 3.5}.",
    "stop_cutterhead": "Stops the cutterhead (and suspends mining).",
    "set_cutterhead_rpm": "Sets the rotation setpoint. Params: {'rpm': 0-8}.",
    "steer": "Moves the steering motors. Params: {'motor': 'A|B|both', "
             "'position_mm': -25..25} or {'delta_mm': x}.",
    "center_steering": "Returns steering motors A and B to center.",
    "set_propulsion_speed": "Track advance setpoint. Params: {'speed_mm_min': 0-60}.",
    "inflate_bladder": "Pressurizes the propulsion bladder. Params: {'target_bar': 0-2.6}.",
    "deflate_bladder": "Depressurizes the bladder (suspends mining).",
    "simulate_fault": "Injects a fault to test alarms. Params: {'fault': "
                      "'cutterhead_overload|propulsion_overload|bladder_underpressure|"
                      "bladder_overpressure|comms_loss|bladder_sensor|"
                      "bladder_sensor_battery|bladder_sensor_signal|hardware_estop|generic'}.",
    "heartbeat": "Refreshes the communications watchdog without altering the machine.",
    "set_bladder_sensor": "Adjusts the wireless sensor. Params: {'battery_percent': 0-100, "
                          "'signal_ms': n, 'link_ok': true|false}.",
    "hardware_estop": "Simulates the PHYSICAL E-stop button. Params: {'pressed': true|false}.",
}


class TelemetryPresenter:
    """Convierte un `ControllerSnapshot` en el JSON obligatorio de TBC.

    El JSON de `/telemetry` es la unica fuente de verdad de la GUI del
    operador: la raiz contiene exactamente las llaves exigidas y el bloque
    `extra` agrupa todos los estados de subsistemas.
    """

    def telemetry(self, snap):
        s = snap.sensors
        return {
            "team": TEAM_NAME,
            "timestamp": int(snap.wall_time),
            "mining": snap.state is State.MINING,
            "chainage": round(s["chainage_m"], 3),
            "easting": round(s["easting_m"], 3),
            "northing": round(s["northing_m"], 3),
            "elevation": round(s["elevation_m"], 3),
            "roll": round(s["roll_deg"], 3),
            "pitch": round(s["pitch_deg"], 3),
            "heading": round(s["heading_deg"], 3),
            "extra": self._extra(snap),
        }

    def _extra(self, snap):
        s = snap.sensors
        now = snap.wall_time
        comms = snap.comms
        gripped = s["bladder_pressure_bar"] >= BLADDER_P_MIN
        return {
            # --- Claves planas (compatibilidad con el dashboard actual) ---
            "cutterhead_rpm": round(s["cutterhead_rpm"], 2),
            "cutterhead_load_percent": round(s["cutterhead_load_pct"], 1),
            "bladder_pressure_bar": round(s["bladder_pressure_bar"], 3),
            "hydroshield_pressure_bar": round(s["hydroshield_pressure_bar"], 3),
            "propulsion_speed_mm_min": round(s["propulsion_speed_mm_min"], 2),
            "bladder_sensor_battery_percent": round(s["bladder_sensor_battery_pct"], 1),
            "bladder_sensor_signal_ms": round(s["bladder_sensor_signal_ms"], 1),

            # --- Maquina de estados ---
            "state": snap.state.value,
            "previous_state": snap.previous_state.value,
            "state_elapsed_s": round(snap.state_elapsed_s, 1),
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now)) + "Z",
            "timestamp_unix_ms": int(now * 1000),
            "sample_hz": TICK_HZ,
            "sample_index": snap.sample_index,

            "cutterhead": {
                "running": snap.cutterhead_running,
                "rpm": round(s["cutterhead_rpm"], 2),
                "rpm_setpoint": round(snap.cutterhead_rpm_setpoint, 2),
                "load_percent": round(s["cutterhead_load_pct"], 1),
                "current_a": round(s["cutterhead_current_a"], 2),
                "torque_nm": round(s["cutterhead_torque_nm"], 1),
                "motor_temp_c": round(s["cutterhead_temp_c"], 1),
                "load_limit_percent": CUTTERHEAD_LOAD_MAX_PCT,
                "current_limit_a": CUTTERHEAD_CURRENT_MAX_A,
                "inhibited": snap.inhibited[SUB_CUTTERHEAD],
                "muck_removal_status": "",
            },
            "propulsion": {
                "enabled": snap.propulsion_enabled,
                "speed_mm_min": round(s["propulsion_speed_mm_min"], 2),
                "speed_setpoint_mm_min": round(snap.propulsion_speed_setpoint, 2),
                "track_left_mm_min": round(s["track_left_mm_min"], 2),
                "track_right_mm_min": round(s["track_right_mm_min"], 2),
                "thrust_force_kn": round(s["thrust_force_kn"], 2),
                "motor_temp_c": round(s["propulsion_temp_c"], 1),
                "load_percent": round(s["propulsion_load_pct"], 1),
                "current_a": round(s["propulsion_current_a"], 2),
                "load_limit_percent": PROPULSION_LOAD_MAX_PCT,
                "current_limit_a": PROPULSION_CURRENT_MAX_A,
                "inhibited": snap.inhibited[SUB_PROPULSION],
                "slip_percent": 0.0,
                "traction_status": "",
            },
            "bladder": {
                "pressure_bar": round(s["bladder_pressure_bar"], 3),
                "setpoint_bar": round(snap.bladder_setpoint_bar, 3),
                "p_min_bar": BLADDER_P_MIN,
                "p_max_bar": BLADDER_P_MAX,
                "inflating": snap.bladder_inflating,
                "relief_valve_open": snap.relief_valve_open,
                "gripped": gripped,
                "wireless_sensor": {
                    "battery_percent": round(s["bladder_sensor_battery_pct"], 1),
                    "signal_ms": round(s["bladder_sensor_signal_ms"], 1),
                    "link_ok": s["bladder_sensor_link_ok"],
                    "battery_critical_percent": BLADDER_SENSOR_BATTERY_CRITICAL_PCT,
                    "signal_lost_ms": BLADDER_SENSOR_SIGNAL_LOST_MS,
                },
            },
            "steering": {
                "motor_a_mm": round(s["steer_a_mm"], 2),
                "motor_b_mm": round(s["steer_b_mm"], 2),
                "motor_a_setpoint_mm": round(snap.steer_a_setpoint_mm, 2),
                "motor_b_setpoint_mm": round(snap.steer_b_setpoint_mm, 2),
                "stroke_limit_mm": STEER_STROKE_MM,
                "yaw_rate_deg_s": round(s["yaw_rate_deg_s"], 4),
                "inhibited": snap.inhibited[SUB_STEERING],
                "angle_error_deg": 0.0,
                "actuator_torque_nm": 0.0,
                "ring_loads_kn": 0.0,
                "estimated_pitch": 0.0,
                "estimated_yaw": 0.0,
            },
            "navigation": {
                "advance_rate_mm_min": round(s["propulsion_speed_mm_min"], 2),
                "chainage_m": round(s["chainage_odometry_m"], 3),
                "mined_length_m": round(snap.mined_length_m, 3),
                "mining_time_s": round(snap.mining_time_s, 1),
                "ground_hardness_index": round(s["ground_hardness_index"], 3),
            },
            "safety": {
                "estop_engaged": snap.estop_engaged,
                "hardware_estop": snap.hardware_estop,
                "active_interlocks": snap.active_interlocks,
                "latched_interlocks": snap.latched_interlocks,
                "interlocks": snap.interlocks,
                "safe_restart": snap.safe_restart,
                "allowed_transitions": snap.allowed_transitions,
                "machine_energized": False,
                "power_loss_detected": False,
                "target_breakthrough_error_m": 0.0,
            },
            "comms": {
                "ok": comms["ok"],
                "seconds_since_last_contact": round(comms["seconds_since_last_contact"], 2),
                "timeout_s": comms["timeout_s"],
                "uplink_ok": comms["uplink_ok"],
                "uplink_failures": comms["uplink_failures"],
                "uplink_sent": comms["uplink_sent"],
                "network_healthy": comms["network_healthy"],
                "io_ok": comms["io_ok"],
                "watchdog_armed_links": comms["watchdog_armed_links"],
                "watchdog_silence_s": round(comms["watchdog_silence_s"], 2),
            },
            "sim": {
                "version": VERSION,
                "adapter": snap.adapter,
                "tick_hz": TICK_HZ,
                "uptime_s": round(snap.uptime_s, 1),
                "commands_received": snap.commands_received,
                "last_command": snap.last_command,
                "injected_faults": snap.injected_faults,
            },
        }

    def command_response(self, result):
        return {
            "ok": result.ok,
            "command": result.command,
            "message": result.message,
            "state": result.state.value,
            "mining": result.mining,
            "estop_engaged": result.estop_engaged,
            "active_interlocks": result.active_interlocks,
            "latched_interlocks": result.latched_interlocks,
            "safe_restart": result.safe_restart,
            "timestamp": result.timestamp,
        }

    def estop_response(self, result):
        """Respuesta de POST /api/plc/parada: exactamente las 4 llaves del
        contrato con el HMI (el detalle completo queda en /telemetry)."""
        return {
            "ok": True,
            "message": ESTOP_HTTP_MESSAGE,
            "state": result.state.value,
            "command": result.command,
        }

    def health(self, snap):
        return {
            "ok": True,
            "team": TEAM_NAME,
            "state": snap.state.value,
            "mining": snap.state is State.MINING,
            "estop_engaged": snap.estop_engaged,
            "latched_interlocks": snap.latched_interlocks,
            "safe_restart": snap.safe_restart,
            "uptime_s": round(snap.uptime_s, 1),
        }

    def api_help(self):
        return {
            "service": "B.I.C.H.O. Supervisory Controller / Digital Twin",
            "team": TEAM_NAME,
            "version": VERSION,
            "architecture": {
                "ports": "IHardwareAdapter (read_sensors, write_actuators, "
                         "check_network_health, is_hardware_estop_triggered)",
                "adapters": "SimulatedHardwareAdapter",
                "core": "TBMController (non-blocking 10 Hz control loop)",
                "presentation": "aiohttp + cliente POST hacia TBC",
            },
            "endpoints": {
                "GET /": "This help.",
                "GET /health": "Short status and safety summary.",
                "GET /telemetry": "Full telemetry in the format required by TBC (10 Hz sample).",
                "POST /command": "Manual/engineering mode: {'command': name, 'params': {...}}.",
                "POST /tbc_endpoint": "Simulated telemetry receiver for The Boring Company.",
                "POST /api/plc/parada": "HMI/PLC emergency stop (alias /api/plc/estop). "
                                        "Payload {'source': 'HMI_Operator'}. Forces E-STOPPED without "
                                        "validating transitions, even with an invalid payload, and responds "
                                        "200 {ok, message, state, command}. Leaving requires 'reset_estop' "
                                        "+ 'arm'.",
            },
            "states": [state.value for state in State],
            "allowed_transitions": {
                state.value: sorted(target.value for target in targets)
                for state, targets in ALLOWED_TRANSITIONS.items()
            },
            "safe_restart": {
                state.value: {
                    "independent_actions_required": len(sequence),
                    "sequence": list(sequence),
                }
                for state, sequence in RESTART_SEQUENCES.items()
            },
            "interlocks": {
                "cutterhead_overload": f"Load > {CUTTERHEAD_LOAD_MAX_PCT:.0f} % or current > "
                                       f"{CUTTERHEAD_CURRENT_MAX_A:.0f} A for "
                                       f"{OVERLOAD_DEBOUNCE_S:.1f} s: stops cutterhead and propulsion (FAULT).",
                "propulsion_overload": f"Load > {PROPULSION_LOAD_MAX_PCT:.0f} % or current > "
                                       f"{PROPULSION_CURRENT_MAX_A:.0f} A for "
                                       f"{OVERLOAD_DEBOUNCE_S:.1f} s: stops propulsion.",
                "bladder_underpressure": f"P_bladder < {BLADDER_P_MIN:.2f} bar: disables propulsion.",
                "bladder_overpressure": f"P_bladder > {BLADDER_P_MAX:.2f} bar: stops inflation, "
                                        "opens the relief valve and disables propulsion (FAULT).",
                "comms_loss": "Watchdog: an armed link (hardware network, operator station in "
                              "READY/MINING, TBC link) with no sign of life for more than "
                              f"{WATCHDOG_TIMEOUT_S:.0f} s: global abort (FAULT).",
                "bladder_sensor": (
                    f"Wireless sensor battery <= {BLADDER_SENSOR_BATTERY_CRITICAL_PCT:.0f} % "
                    f"or signal >= {BLADDER_SENSOR_SIGNAL_LOST_MS:.0f} ms: machine to FAULT."
                ),
            },
            "commands": COMMAND_HELP,
            "startup_sequence": [
                "power_on", "arm", "set_state READY", "inflate_bladder",
                "start_cutterhead", "set_state MINING",
            ],
        }


# --- Utilidades HTTP -----------------------------------------------------

CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type, Authorization",
}

MAX_BODY_BYTES = 1_000_000

HTTP_STATUS_BY_OUTCOME = {
    CommandOutcome.ACCEPTED: 200,
    CommandOutcome.REJECTED: 409,
    CommandOutcome.INVALID: 400,
}


def json_response(payload, status=200):
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    return web.Response(
        body=body,
        status=status,
        content_type="application/json",
        charset="utf-8",
        headers={"Cache-Control": "no-store"},
    )


async def read_json_body(request, required=True):
    """Devuelve (dict, None) o (None, mensaje_de_error)."""
    raw = await request.read()
    if not raw.strip():
        if required:
            return None, "Empty body: a JSON object is expected."
        return {}, None
    try:
        body = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"Invalid JSON: {exc}"
    if not isinstance(body, dict):
        return None, "The JSON must be an object, for example {\"command\": \"arm\"}."
    return body, None


def sanitize_source(raw, default=ESTOP_SOURCE_DEFAULT, limit=ESTOP_SOURCE_MAX_CHARS):
    """Texto de origen NO confiable -> una sola linea imprimible y acotada.

    El campo 'source' viene de la red y termina en el log de auditoria:
    se eliminan saltos de linea/caracteres de control (evita falsificar
    lineas de log) y se limita la longitud.
    """
    printable = "".join(ch if ch.isprintable() else " " for ch in str(raw))
    return " ".join(printable.split())[:limit] or default


if web is not None:

    @web.middleware
    async def http_envelope(request, handler):
        """CORS, preflight OPTIONS y errores siempre en JSON."""
        if request.method == "OPTIONS":
            return web.Response(
                status=204,
                headers={**CORS_HEADERS, "Access-Control-Max-Age": "86400"},
            )
        try:
            response = await handler(request)
        except web.HTTPException as exc:
            if exc.status == 404:
                response = json_response({
                    "ok": False,
                    "error": f"Route not found: {request.path_qs}",
                    "hint": "Use GET /telemetry, GET / or POST /command.",
                }, 404)
            else:
                response = json_response(
                    {"ok": False, "error": exc.reason or "HTTP error"}, exc.status
                )
        except Exception as exc:                    # nunca tumbar el servidor
            log_warn(f"Internal error in {request.method} {request.path}: {exc!r}")
            response = json_response(
                {"ok": False, "error": "Internal server error."}, 500
            )
        response.headers.update(CORS_HEADERS)
        return response


class TelemetryApi:
    """Handlers REST. Solo traducen HTTP <-> API publica del nucleo."""

    def __init__(self, controller, presenter):
        self._controller = controller
        self._presenter = presenter
        self.tbc_packets = 0

    async def root(self, request):
        return json_response(self._presenter.api_help())

    async def health(self, request):
        return json_response(self._presenter.health(self._controller.snapshot()))

    async def telemetry(self, request):
        self._controller.touch_operator()
        return json_response(self._presenter.telemetry(self._controller.snapshot()))

    async def command(self, request):
        body, error = await read_json_body(request)
        if error:
            log_warn(f"POST /command rejected: {error}")
            return json_response(
                {"ok": False, "error": error, "commands": sorted(COMMAND_HELP)}, 400
            )
        command = body.get("command", body.get("cmd", body.get("action")))
        if command is None:
            return json_response({
                "ok": False,
                "error": "Missing key 'command'.",
                "example": {"command": "set_state", "params": {"state": "READY"}},
                "commands": sorted(COMMAND_HELP),
            }, 400)
        params = body.get("params")
        if not isinstance(params, dict):
            params = {
                key: value
                for key, value in body.items()
                if key not in ("command", "cmd", "action", "params")
            }
        result = self._controller.execute(command, params)
        return json_response(
            self._presenter.command_response(result), HTTP_STATUS_BY_OUTCOME[result.outcome]
        )

    async def tbc_endpoint(self, request):
        """Receptor simulado de The Boring Company."""
        body, error = await read_json_body(request)
        if error:
            return json_response({"status": "error", "message": error}, 400)
        self.tbc_packets += 1
        missing = [key for key in TBC_ROOT_KEYS if key not in body]
        if missing:
            log_warn(f"TBC packet #{self.tbc_packets} incomplete, missing: {', '.join(missing)}")
        elif self.tbc_packets == 1 or self.tbc_packets % 10 == 0:
            log_net(
                f"📦 Telemetry packet #{self.tbc_packets} received by the simulated TBC "
                f"receiver (team={body.get('team')}, chainage={body.get('chainage')}, "
                f"mining={body.get('mining')})."
            )
        return json_response({"status": "success", "message": "Telemetry received by TBC"})

    async def plc_estop(self, request):
        """POST /api/plc/parada  {"source": "HMI_Operator"}.

        FAIL-SAFE: una parada de emergencia nunca se rechaza por culpa del
        payload. Un cuerpo vacio, mal formado, no-objeto o ilegible solo
        degrada el campo 'source' a ESTOP_SOURCE_DEFAULT; la maquina se
        detiene igual y se responde 200.
        """
        source = await self._estop_source(request)
        result = self._controller.engage_estop(source=source)
        return json_response(self._presenter.estop_response(result))

    @staticmethod
    async def _estop_source(request):
        """Origen declarado por el cliente, saneado. Nunca lanza excepcion."""
        try:
            body, error = await read_json_body(request, required=False)
        except Exception as exc:                    # cuerpo ilegible o demasiado grande
            body, error = None, repr(exc)
        if error:
            log_warn(
                f"POST /api/plc/parada with invalid payload ({error}); "
                "the emergency stop is executed anyway."
            )
            return ESTOP_SOURCE_DEFAULT
        return sanitize_source(body.get("source", body.get("origin", ESTOP_SOURCE_DEFAULT)))


def create_app(controller, presenter):
    """Construye la aplicacion aiohttp (servidor local asincrono)."""
    api = TelemetryApi(controller, presenter)
    app = web.Application(middlewares=[http_envelope], client_max_size=MAX_BODY_BYTES)

    def add(method, path, handler):
        for variant in {path, path.rstrip("/") or "/", path.rstrip("/") + "/"}:
            if method == "GET":
                app.router.add_get(variant, handler)     # incluye HEAD
            else:
                app.router.add_post(variant, handler)

    add("GET", "/", api.root)
    add("GET", "/help", api.root)
    for alias in ("/health", "/status", "/state"):
        add("GET", alias, api.health)
    add("GET", "/telemetry", api.telemetry)
    add("POST", "/command", api.command)
    add("POST", "/tbc_endpoint", api.tbc_endpoint)
    add("POST", "/api/plc/parada", api.plc_estop)
    add("POST", "/api/plc/estop", api.plc_estop)
    app["api"] = api
    return app


class TBCUplinkClient:
    """Cliente TBC: tarea asincrona que hace POST de la telemetria a un
    servidor externo a >= 0.1 Hz (Regla 7 de TBC)."""

    MIN_HZ = 0.1
    REQUEST_TIMEOUT_S = 5.0

    def __init__(self, controller, presenter, url, hz, token=None):
        if hz < self.MIN_HZ:
            raise ValueError(
                f"The uplink cannot be slower than {self.MIN_HZ} Hz (TBC requirement)."
            )
        self._controller = controller
        self._presenter = presenter
        self.url = url
        self.hz = float(hz)
        self.period_s = 1.0 / self.hz
        self._token = token

    async def run(self):
        log_net(
            f"TBC client active -> {self.url} at {self.hz:g} Hz "
            f"(every {self.period_s:.1f} s; TBC minimum {self.MIN_HZ} Hz)."
        )
        headers = {"Authorization": f"Bearer {self._token}"} if self._token else {}
        timeout = aiohttp.ClientTimeout(total=min(self.REQUEST_TIMEOUT_S, 0.8 * self.period_s))
        loop = asyncio.get_running_loop()
        async with aiohttp.ClientSession(timeout=timeout, headers=headers) as session:
            next_send = loop.time()
            while True:
                payload = self._presenter.telemetry(self._controller.snapshot())
                try:
                    async with session.post(self.url, json=payload) as response:
                        await response.read()
                        ok = 200 <= response.status < 300
                        self._controller.register_uplink_result(ok, f"HTTP {response.status}")
                except (aiohttp.ClientError, asyncio.TimeoutError, OSError) as exc:
                    self._controller.register_uplink_result(False, repr(exc))
                next_send += self.period_s
                delay = next_send - loop.time()
                if delay <= 0.0:
                    next_send = loop.time()
                    delay = 0.0
                await asyncio.sleep(delay)


# =========================================================================
# RAIZ DE COMPOSICION (unico lugar que conoce todas las capas)
# =========================================================================

def print_banner(args, controller, uplink_url):
    shown_host = args.host or "0.0.0.0"
    base = f"http://localhost:{args.port}"
    print("=" * 72)
    print(f"🚀  B.I.C.H.O. SUPERVISORY CONTROLLER  v{VERSION}   |   {TEAM_NAME}")
    print("=" * 72)
    print("🏗️   Architecture: Ports -> Adapters -> Core -> API (aiohttp)")
    print(f"🔌  Injected hardware ....... {controller.adapter_name}")
    print(f"🌐  Listening on {shown_host}:{args.port}   (loop and telemetry at {TICK_HZ:.0f} Hz)")
    print(f"📡  Telemetry ............... GET  {base}/telemetry")
    print(f"🎛️   Manual/engineering mode . POST {base}/command")
    print(f"🛑  Simulated TBC receiver .. POST {base}/tbc_endpoint")
    print(f"🚨  HMI emergency stop ...... POST {base}/api/plc/parada  {{\"source\": \"HMI_Operator\"}}")
    print(f"📖  API help ................ GET  {base}/")
    print(f"🐕  Network watchdog ........ {controller.watchdog_timeout_s:.0f} s without sign of life => FAULT")
    if uplink_url:
        print(f"⬆️   TBC client .............. POST {uplink_url} at {args.uplink_hz:g} Hz")
    else:
        print("⚠️   TBC client DISABLED (--no-uplink): violates TBC Rule 7.")
    print("-" * 72)
    print(f"🔐  Initial state: {controller.state.value}")
    print("    Start-up sequence (each step is a POST /command):")
    print("      1) power_on          2) arm                 -> IDLE")
    print("      3) set_state READY   4) inflate_bladder     -> bladder gripping")
    print("      5) start_cutterhead  6) set_state MINING    -> mining")
    print("    Safe Restart: E-STOPPED / POWER OFF / FAULT require 2 actions.")
    print("-" * 72)
    print("    Example (PowerShell):")
    print(f"      Invoke-RestMethod {base}/command -Method Post "
          "-ContentType 'application/json' -Body '{\"command\":\"power_on\"}'")
    print("    Example (curl):")
    print(f"      curl -X POST {base}/command -H \"Content-Type: application/json\" "
          "-d \"{\\\"command\\\":\\\"estop\\\"}\"")
    print("-" * 72)
    print("⚠️   Press Ctrl+C to shut down the controller.")
    print("=" * 72, flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="B.I.C.H.O. supervisory controller / digital twin - UDD Tunnel Lab."
    )
    parser.add_argument("--host", default="", help="Listening interface (default: all).")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port.")
    parser.add_argument(
        "--start-state",
        default="POWER OFF",
        help="Initial state (POWER OFF by default; use IDLE for GUI development).",
    )
    parser.add_argument(
        "--comms-timeout",
        type=float,
        default=WATCHDOG_TIMEOUT_S,
        help="Seconds without sign of life on an armed link before global abort (def. 10).",
    )
    parser.add_argument(
        "--uplink-url", default=None,
        help="TBC server URL (default: local simulated receiver /tbc_endpoint).",
    )
    parser.add_argument(
        "--uplink-hz", type=float, default=1.0,
        help="POST frequency to TBC in Hz (default 1; TBC minimum 0.1).",
    )
    parser.add_argument("--uplink-token", default=None, help="Bearer token for the uplink.")
    parser.add_argument(
        "--no-uplink", action="store_true",
        help="Disables the TBC client (development only: violates Rule 7).",
    )
    parser.add_argument("--seed", type=int, default=None, help="Simulated noise seed.")
    return parser.parse_args(argv)


def _report_task_failure(task):
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        log_warn(f"Task '{task.get_name()}' terminated with error: {exc!r}")


async def serve(args, start_state):
    """Compone las capas (inyeccion de dependencias) y las ejecuta."""
    # --- Adaptador (capa 2) inyectado en el nucleo (capa 3) ---
    hardware = SimulatedHardwareAdapter(rng=random.Random(args.seed))
    controller = TBMController(
        hardware,
        watchdog_timeout_s=args.comms_timeout,
        start_state=start_state,
    )
    controller.step()                          # primer sample antes de abrir el puerto

    # --- Presentacion (capa 4) ---
    presenter = TelemetryPresenter()
    app = create_app(controller, presenter)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    try:
        site = web.TCPSite(runner, args.host or "0.0.0.0", args.port)
        await site.start()
    except OSError as exc:
        await runner.cleanup()
        print(f"Could not open port {args.port}: {exc}")
        return 1

    uplink_url = None
    if not args.no_uplink:
        uplink_url = args.uplink_url or f"http://127.0.0.1:{args.port}/tbc_endpoint"
    print_banner(args, controller, uplink_url)

    tasks = [asyncio.create_task(controller.run(), name="control-loop")]
    if uplink_url:
        client = TBCUplinkClient(
            controller, presenter, uplink_url, args.uplink_hz, token=args.uplink_token
        )
        tasks.append(asyncio.create_task(client.run(), name="tbc-uplink"))
    for task in tasks:
        task.add_done_callback(_report_task_failure)

    try:
        await asyncio.Event().wait()           # hasta Ctrl+C
    except asyncio.CancelledError:
        print()
        log_info("Shutdown requested by the operator (Ctrl+C).")
        raise
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await runner.cleanup()
        snap = controller.snapshot()
        log_info(
            f"Final summary: state={snap.state.value}, "
            f"chainage={snap.sensors['chainage_odometry_m']:.3f} m, "
            f"commands={snap.commands_received}, "
            f"mining time={snap.mining_time_s:.1f} s."
        )
        print("🛑 B.I.C.H.O. controller stopped.")
    return 0


def main(argv=None):
    configure_console()
    args = parse_args(argv)

    if aiohttp is None:
        print("Missing dependency 'aiohttp'. Install it with:\n"
              "    pip install -r requirements.txt")
        return 3

    start_state = parse_state(args.start_state)
    if start_state is None:
        print(f"Invalid initial state: '{args.start_state}'. "
              f"Valid: {', '.join(s.value for s in State)}")
        return 2
    if start_state in (State.MINING, State.FAULT, State.E_STOPPED):
        print(f"The initial state cannot be {start_state.value}: the machine must "
              "start in POWER OFF, IDLE or READY.")
        return 2
    if not args.no_uplink and args.uplink_hz < TBCUplinkClient.MIN_HZ:
        print(f"The uplink cannot be slower than {TBCUplinkClient.MIN_HZ} Hz "
              "(TBC requirement).")
        return 2
    if args.comms_timeout <= 0.0:
        print("--comms-timeout must be greater than 0.")
        return 2

    try:
        return asyncio.run(serve(args, start_state))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
