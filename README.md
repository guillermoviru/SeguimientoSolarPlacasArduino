### Descripción general (v2.4.2)

* **Propósito** – Arduino Opta controla la inclinación de paneles solares.  
* **Entradas**   
  * Inclinómetro BWK-217 (Modbus-TCP)  
  * End-stop vertical (A0)  
  * Botón USER = parada de emergencia  
* **Proceso**   
  1. Sincroniza hora NTP *(pool-esp)*  
  2. Calcula elevación solar (`SolarCalculator`)  
  3. Lee ángulo real, filtra (0 .7 / 0 .3)  
  4. FSM decide **SUBIR / BAJAR / ALINEADO**  
  5. Verifica seguridad: límites, timeout, end-stop  
* **Salidas** – Relés D1 (SUBIR) y D2 (BAJAR) + LEDs D7/D9.  
* **Seguridad** – Watchdog 8 s, reintento (10 min máx 3) → FIN_CARRERA_V / MODO_ESPERA.  
* **Logs** – Serial con niveles INFO / ACTION / WARNING / ERROR.
