# PRD: Backend Simulado B.I.C.H.O. (Digital Twin)

## 1. Contexto del Proyecto

Este servidor en Python actúa como el gemelo digital de la tuneladora a escala funcional del equipo UDD Tunnel Lab para participar en la Not-a-Boring Competition[cite: 4]. El objetivo es construir una arquitectura modular que sirva para probar la interfaz gráfica (GUI) del operador de forma remota[cite: 2, 3].

## 2. Requerimientos Core (Cumplimiento TBC)

- **Frecuencia:** El servidor HTTP debe ser capaz de procesar y enviar telemetría a una frecuencia mínima de 0.1 Hz[cite: 1, 2, 3].
- **Posicionamiento:** Debe calcular y reportar la posición 3D y orientación 3D en todo momento, incluyendo los 6 grados de libertad (Easting, Northing, Elevation, Roll, Pitch, Heading)[cite: 1, 2, 3].
- **Operación Remota:** La tuneladora debe operarse de forma 100% remota desde la superficie, sin humanos a bordo[cite: 1, 3].

## 3. Arquitectura de Estados y Seguridad

El simulador debe basarse en una máquina de estados para evitar transiciones inseguras y garantizar que no se pase directamente de un estado inseguro a excavar[cite: 3]. 

### Estados Permitidos

- `POWER OFF`[cite: 2, 3]
- `E-STOPPED`[cite: 2, 3]
- `IDLE`[cite: 2, 3]
- `READY`[cite: 2, 3]
- `MINING`[cite: 2, 3]
- `FAULT`[cite: 2, 3]

### Reglas de Seguridad Estrictas

- **Condición de Reinicio Seguro (Safe Restart):** Retornar a la máquina a un estado operativo (running state) desde una parada de emergencia (E-STOPPED) o pérdida de energía debe requerir obligatoriamente al menos 2 acciones independientes[cite: 2, 3].
- **Gatillos de Falla (Interlocks Automáticos):** La máquina debe pasar a estado `FAULT` o inhabilitar subsistemas automáticamente bajo las siguientes condiciones simuladas:
  - Sobrecorriente o sobrecarga en el cabezal de corte (Cutterhead overload): Detiene el cabezal y la propulsión[cite: 3].
  - Presión de la vejiga de propulsión muy baja `P_bladder < P_min`): Deshabilita la propulsión[cite: 3].
  - Sobrepresión en la vejiga `P_bladder > P_max`): Detiene el inflado, activa el alivio de presión y deshabilita la propulsión[cite: 3].
  - Pérdida de comunicación de red: Aborto global y detención de la actuación[cite: 3].

## 4. Endpoints Requeridos (API REST local)

### GET /telemetry

- Retorna un JSON con el estado actual de la máquina[cite: 2].
- Formato obligatorio exigido: Debe incluir las llaves `"team"`, `"timestamp"`, `"mining"`, `"chainage"`, `"easting"`, `"northing"`, `"elevation"`, `"roll"`, `"pitch"`, `"heading"` y un bloque `"extra"`[cite: 2].

 *Comportamiento dinámico: Si el estado es* `MINING`*, las variables de navegación (*chainage*, *easting*, *northing*) deben avanzar matemáticamente simulando el progreso[cite: 2]. Si no está en `MINING`, se quedan estáticas.

- El bloque `"extra"` debe incluir métricas operativas simuladas (presión del hydroshield, RPM del cabezal, velocidad de los motores tipo oruga de propulsión, etc.)[cite: 2, 3].

### POST /command (Modo Manual/Ingeniería)

- Recibe instrucciones JSON para alterar el comportamiento del simulador en un modo de control manual[cite: 3].
- Comandos aceptados: Cambiar estado general (ej. solicitar paso de IDLE a READY), arrancar/detener el cabezal de corte, mover motores de dirección A/B, o simular un fallo crítico para probar alarmas[cite: 3].

## 5. Requisitos de Código

- Escrito en Python 3 puro, sin librerías externas complejas.
- Uso de programación orientada a objetos (crear una clase estructurada para la máquina de estados).
- Incluir logs claros en la consola cuando se recibe un comando, se activa un Interlock de seguridad o cambia un estado general.

