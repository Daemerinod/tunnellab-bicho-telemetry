import json
import time
import math
from http.server import BaseHTTPRequestHandler, HTTPServer

tiempo_inicio = time.time()

class TelemetryHandler(BaseHTTPRequestHandler):
    
    def do_GET(self):
        if self.path == '/telemetry':
            # Primero generamos los datos y los preparamos
            data = self.generar_datos_telemetria()
            respuesta_json = json.dumps(data).encode('utf-8')

            # Enviamos el encabezado HTTP
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*') 
            # ¡ESTA ES LA LÍNEA MÁGICA QUE FALTABA! Le dice al navegador cuándo dejar de cargar
            self.send_header('Content-Length', str(len(respuesta_json)))
            self.end_headers()
            
            # Enviamos los datos
            self.wfile.write(respuesta_json)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/tbc_endpoint':
            respuesta = {"status": "success", "message": "Telemetry received by TBC"}
            respuesta_json = json.dumps(respuesta).encode('utf-8')

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(respuesta_json)))
            self.end_headers()

            self.wfile.write(respuesta_json)
            print("📦 [LOG] Paquete POST recibido exitosamente simulando servidor de TBC.")
        else:
            self.send_response(404)
            self.end_headers()

    def generar_datos_telemetria(self):
        tiempo_transcurrido = time.time() - tiempo_inicio
        avance_actual = 12.45 + (tiempo_transcurrido * 0.01) 
        vibracion = math.sin(tiempo_transcurrido) * 0.5 

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
        pass

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