
---

## 2. Pines y cableado

| Elemento                 | Pin Opta | Modo  |
|--------------------------|----------|-------|
| Relé **SUBIR**           | **D1**   | OUTPUT |
| Relé **BAJAR**           | **D2**   | OUTPUT |
| LED **SUBIR**            | **D7**   | OUTPUT |
| LED **BAJAR**            | **D9**   | OUTPUT |
| LED **ERROR**            | **D8**   | OUTPUT |
| End-stop vertical (NC)   | **A0**   | INPUT_PULLUP |
| Botón de emergencia      | **BTN_USER** | INPUT_PULLUP |
| Ethernet RJ-45           | Puerto ETH | estática 192.168.1.100 |
| Inclinómetro BWK-217     | LAN → RS485 | IP 192.168.1.150 · ID 2 |

*El end-stop debe abrirse (HIGH) cuando se alcanza el tope; el código invierte la lógica si se usa contacto NA.*

---

## 3. Flujo de operación (resumen)

1. **Arranque**  
   * Ethernet fija → `192.168.1.100`  
   * Watchdog mbed 8 s  
   * Conexión a BWK-217 → lee ángulo inicial  
   * Sincroniza hora NTP (3 intentos)

2. **Bucle 1 s**  
   * Refresca hora NTP cada 5 min  
   * Recalcula elevación solar → nuevo objetivo si Δ > 0.5 °  
   * Lee inclinómetro (filtro 0.7 / 0.3)  
   * **FSM** decide:  
     * `SUBIR` (D1 ON, D7 ON)  
     * `BAJAR` (D2 ON, D9 ON)  
     * `ALINEADO` (off)  
   * Histéresis dinámica (¼ de Δ, 0.5°…5°)

3. **Seguridad**  
   * Límites software `ANGULO_MIN/MAX (-0.5…85°)`  
   * End-stop vertical -- bloquea SUBIR y genera `FIN_CARRERA_V`  
   * Timeout movimiento 15 s con auto-reset si avanza ≥ 0.5°  
   * 3 reintentos → si falla pasa a `MODO_ESPERA` y fuerza posición segura (45°)  
   * Botón USER = emergencia (corta relés y entra en `ERROR_SEGURIDAD`)  

4. **Registro**  
   * Salida serial con TIMESTAMP + nivel (`INFO · ACTION · WARNING · ERROR`)  
   * Cada 5 s muestra estado, ángulo y próximo reintento.  

---

## 4. Compilar y cargar

1. **Arduino IDE ≥ 2.2.0**  
2. Instalar **Arduino Mbed OS Opta Boards v 4.3.1**  
3. Añadir librerías: `ArduinoModbus`, `NTPClient`, `SolarCalculator`, `Ethernet`  
4. Cargar el sketch `BrujulaAnamometro.ino` → placa *Opta* → 115 200 baudios.

---

## 5. Extensibilidad

* Añadir segundo end-stop (mínimo) → copiar lógica `ENDSTOP_MIN_PIN`.  
* Sensor de corriente → lanzar `ERROR_SEGURIDAD` si I > umbral.  
* Encoder absoluto → sustituir inclinómetro.  
* MQTT / Modbus-RTU slave → fácil: el código está modularizado.

---

© 2024 Guillermo ____ – MIT License
