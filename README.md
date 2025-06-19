Descripción general del programa - v2.4.2
Área	Resumen
Propósito	Controlar la inclinación de una plataforma/fila de paneles solares mediante un Arduino Opta. La posición real se mide con un inclinómetro (BWK-217, Modbus-TCP). El sistema calcula el ángulo solar, corrige la posición con relés “SUBIR / BAJAR” y aplica múltiples capas de seguridad.
Arquitectura principal	1. Inicialización
 • Ethernet estática → IP 192.168.1.100.
 • Sincroniza hora (NTP, pool-esp).
 • Conecta al esclavo Modbus TCP (IP 150, id 2).
 • Lee ángulo inicial.
2. Bucle (1 s)
 • Vigila botón de emergencia (BTN_USER).
 • Re-sincroniza la hora cada 5 min → recalcula altura solar.
 • Lee inclinómetro (ﬁltro exponencial 0.7 / 0.3).
 • FSM → decide SUBIR / BAJAR / ALINEADO.
 • Verifica seguridad (límite mecánico, timeout, end-stop).
 • Registra eventos cada 5 s.
Cálculo solar	Usa SolarCalculator → calcHorizontalCoordinates() para obtener elevación (-10 … +95°) y la establece como nuevo ángulo-objetivo si la diferencia > 0.5 grados.
Control de motores	Histéresis dinámica (¼ de la diferencia, min 0.5° / max 5°).
Relés D1 (SUBIR) y D2 (BAJAR) + LEDs D7/D9 como indicadores.
Seguridad	• Watchdog mbed (8 s).
• Botón USER = parada de emergencia.
• End-stop vertical en A0 (HIGH = pulsado).
 – Bloquea nuevas órdenes SUBIR.
 – Si se activa durante la subida → estado FIN_CARRERA_V.
• Límites ANGULO_MIN/MAX (-0.5 … 85°).
• Timeout de movimiento (15 s) con auto-reset al detectar avance≥0.5°.
Gestión de fallos	• NTP / Modbus: pasa a estado de error y enciende LED_ERROR.
• “Reintento pendiente” → espera 10 min y reintenta; tras 3 fallos entra en ERROR_MOVIMIENTO → fuerza posición segura (≈ 45°) y pasa a MODO_ESPERA.
Registro (Serial)	Time-stamp local y milisegros desde arranque.
Niveles: INFO, ACTION, WARNING, ERROR (⛔).
Configuración sencilla	Constantes al principio: IPs, tolerancias, velocidad, tiempo de espera, etc.
Extensibilidad	El código está modularizado: helpers para NTP, Modbus, seguridad. Se puede añadir otro end-stop o sensórica de corriente sin alterar la lógica central.
