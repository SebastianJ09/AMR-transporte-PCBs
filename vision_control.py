import cv2
import time
import paho.mqtt.client as mqtt

# ================= CONFIGURACIÓN =================
# 0 usa la cámara web de tu PC para probar ahora mismo.
# Cuando conectes el ESP32, usa el puerto 82 dedicado a Python/OpenCV:
STREAM_URL = "http://192.168.4.1:82/stream"

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_COMANDOS = "amr/comandos/movimiento"

# ================= CLIENTE MQTT =================
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    print("[MQTT] Conectado exitosamente al broker Mosquitto.")
except Exception as e:
    print(f"[MQTT] Error conectando a Mosquitto: {e}")
    print("[MQTT] Verifica que el servicio Mosquitto esté corriendo en services.msc.")

# ================= CAPTURA Y VISIÓN =================
print(f"[VISIÓN] Abriendo cámara: {STREAM_URL}...")
cap = cv2.VideoCapture(STREAM_URL)
qr_detector = cv2.QRCodeDetector()

ultimo_envio = 0
INTERVALO_ENVIO = 1.0  # Segundos entre publicaciones para no saturar la red

while True:
    ret, frame = cap.read()
    if not ret:
        print("[VISIÓN] Esperando señal de video...")
        time.sleep(0.5)
        continue

    # Detección de código QR
    data, bbox, _ = qr_detector.detectAndDecode(frame)

    if bbox is not None and len(bbox) > 0:
        puntos = bbox[0].astype(int)
        n = len(puntos)
        for j in range(n):
            cv2.line(frame, tuple(puntos[j]), tuple(puntos[(j + 1) % n]), (0, 255, 0), 2)

        if data:
            cv2.putText(frame, f"QR: {data}", (puntos[0][0], puntos[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if time.time() - ultimo_envio > INTERVALO_ENVIO:
                comando = f"DESTINO:{data}"
                client.publish(TOPIC_COMANDOS, comando)
                print(f"[MQTT ENVIADO] -> {comando}")
                ultimo_envio = time.time()

    cv2.imshow("AMR - Vision Central y Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
client.loop_stop()
client.disconnect()
print("[SISTEMA] Programa finalizado.")