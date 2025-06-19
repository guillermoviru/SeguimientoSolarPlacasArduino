# SeguimientoSolarPlacasArduino
Opta PLC controla un seguidor solar: obtiene hora NTP, calcula elevación solar, lee inclinómetro vía Modbus-TCP y mueve motores con relés SUBIR/BAJAR hasta alinear la plataforma. Incluye watchdog, botón de emergencia, final-de-carrera vertical, límites de ángulo y reintentos seguros, registrando eventos por serie.
