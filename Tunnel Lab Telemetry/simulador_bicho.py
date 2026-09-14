import json
import time
import math
from http.server import BaseHTTPRequestHandler, HTTPServer

tiempo_inicio = time.time()

class TelemetryHandler(BaseHTTPRequestHandler):
    
    # 1. MÉTODO GET: Para que el Dashboard (Frontend) de UDD Tunnel Lab lea los datos en tiempo real.
    def do_GET(self):
        if self.path == '/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*') 
            self.end_headers()
            
            data = self.generar_datos_telemetria()
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    # 2. MÉTODO POST: Simula el endpoint que exige The Boring Company en las reglas.
    def do_POST(self):
        if self.path == '/tbc_endpoint':
            # Aquí TBC evaluará las credenciales de autenticación en la competencia
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            respuesta = {"status": "success", "message": "Telemetry received by TBC"}
            self.wfile.write(json.dumps(respuesta).encode('utf-8'))
            print("📦 [LOG] Paquete POST recibido exitosamente simulando servidor de TBC.")
        else:
            self.send_response(404)
            self.end_headers()

    # Función central que genera el JSON oficial de la competencia
    def generar_datos_telemetria(self):
        tiempo_transcurrido = time.time() - tiempo_inicio
        avance_actual = 12.45 + (tiempo_transcurrido * 0.01) 
        vibracion = math.sin(tiempo_transcurrido) * 0.5 

        # Estructura JSON exacta requerida por TBC (Regla 8.c)
        return {
          "team": "UDD Tunnel Lab",
          "timestamp": int(time.time()),
          "mining": True,
          "chainage": round(avance_actual, 3),
          "easting": round(342118.64 + (tiempo_transcurrido * 0.005), 2),
          "northing": round(6296004.21 + (tiempo_transcurrido * 0.005), 2),
          "elevation": -42.180,
          "roll": round(0.62 + vibracion, 3),
          "pitch": round(-1.18 + (vibracion * 0.5), 3),
          "heading": 118.44,
          "extra": {
             "cutterhead_rpm": round(3.42 + (math.cos(tiempo_transcurrido)*0.1), 2),
             "bladder_pressure_bar": 1.82,
             "hydroshield_pressure_bar": 2.45,
             "propulsion_speed_mm_min": 41.9,
             "cutterhead_load_percent": 88
          }
        }

    def log_message(self, format, *args):
        pass # Silencia los logs de consola para mantenerla limpia

def iniciar_servidor(puerto=8000):
    direccion = ('', puerto)
    servidor = HTTPServer(direccion, TelemetryHandler)
    print("=====================================================")
    print(f"🚀 Gemelo Digital B.I.C.H.O. corriendo exitosamente.")
    print(f"📡 Dashboard local lee en: http://localhost:{puerto}/telemetry (GET)")
    print(f"🛑 Endpoint simulado TBC: http://localhost:{puerto}/tbc_endpoint (POST)")
    print("⚠️  Presiona Ctrl+C para apagar.")
    print("=====================================================")
    servidor.serve_forever()

if __name__ == '__main__':
    iniciar_servidor()