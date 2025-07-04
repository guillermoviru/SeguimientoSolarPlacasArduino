/* =========================================================
   INCLINÓMETRO + MODBUS + NTP + CONTROL MOTORES – v2.5.0-ES
   Arduino Opta (Mbed-OS core)        (c) 2024
   ========================================================= */

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <NTPClient.h>
#include <ArduinoModbus.h>
#include <solarfunctions.h>   
#include <TimeLib.h>      
#include "mbed.h"

/* ---------- BRÚJULA: auto-calibración ---------- */
float  azSurRef         = NAN;      // referencia Sur capturada al inicio
bool   brujulaCalibrada = false;    // pasa a true cuando se promedia
const  uint16_t MUESTRAS_CALIB = 20;    // lecturas para el promedio
const  uint32_t TIEMPO_CALIB_MS = 3000; // ventana de 3 s

/* ---------- AJUSTES HMI ---------- */
const int    CORR_TIEMPO_SEG = 0;   // adelanta 2 min 07 s
const double DESFASE_AZIMUT  = 0;   // offset opcional
const double DESFASE_CENIT   = 0.00;
const double CENIT_MAX = 40.0;   // límite superior de apertura
/* --------------------------------- */

/* ---------- ENUMERACIONES ---------- */
enum EstadoSistema {
    INICIALIZANDO, OPERATIVO, MOVIENDO_ARRIBA, MOVIENDO_ABAJO, ALINEADO,
    ERROR_NTP, ERROR_MODBUS, ERROR_SEGURIDAD, ERROR_MOVIMIENTO,
    REINTENTO_PENDIENTE, MODO_ESPERA, ENDSTOP_VERTICAL
};

enum EstadoViento { VIENTO_OK, VIENTO_TORMENTA, VIENTO_ALERTA };
volatile EstadoViento estadoViento = VIENTO_OK;

enum EstadoAzimut { AZ_INIT, MOV_AZ_IZQ, MOV_AZ_DER, AZ_ALINEADO, AZ_REINTENTO, AZ_ERROR_MOV };
volatile EstadoAzimut estadoAz = AZ_INIT;

// Estados para posición segura no bloqueante
enum PosicionSegura { PS_INACTIVO, PS_SUBIENDO, PS_BAJANDO, PS_COMPLETADO };
volatile PosicionSegura estadoPosicion = PS_INACTIVO;

/* ---------- PROTOTIPOS ---------- */
void registrarEvento(const char* tipo, const char* mensaje, bool critico = false);
bool inicializarEthernet();
bool inicializarModbus();
bool sincronizarHoraNTP();
float leerAnguloActual();
void controlarSalidas(bool&, bool&);
void actualizarAnguloSolar(uint32_t now);
void verificarSeguridadMecanica(float);
void cambiarEstado(EstadoSistema);
const char* nombreEstado(EstadoSistema);
void manejarErrorMovimiento();
void reiniciarSistema();
void manejarPosicionSegura(uint32_t now);
float leerViento();
void actualizarEstadoViento(float);
void aplicarProteccionViento();
const char* textoEstadoViento(EstadoViento e);
bool leerAzimutSEC345(float &azimut);
void cambiarEstadoAz(EstadoAzimut);
bool configurarModoPreguntaRespuesta();
void inicializarBreaker();
void vigilarBreaker();
float decodificarAzimut(uint8_t b0, uint8_t b1, uint8_t b2);

/* ---------- CONFIGURACIÓN ---------- */
#define VERSION_FIRMWARE "2.5.0-ES"
#define BAUDIOS_SERIAL   115200

/* Macros de log optimizadas */
#define LOG_INFO(m) registrarEvento("💡 INFO", m)
#define LOG_ACCION(m)  registrarEvento("⚙️ ACCIÓN", m)
#define LOG_ESTADO(m)  registrarEvento("🔄 ESTADO", m)
#define LOG_MEDIDA(m)  registrarEvento("📊 MEDIDA", m)
#define LOG_ADVERT(m)  registrarEvento("⚠️ AVISO", m, true)
#define LOG_ERROR_(m)  registrarEvento("⛔ ERROR", m, true)

/* ---------- RED ---------- */
byte mac[] = {0x00, 0x1A, 0xB6, 0x03, 0x2D, 0xA1};
const uint8_t SET_QA_MODE[] = {0x77, 0x05, 0x00, 0x0C, 0x00, 0x11};
const IPAddress ipSEC345(192, 168, 1, 150);
const uint16_t puertoModbus = 502;
const uint8_t idSEC345 = 1;      
const IPAddress ip(192,168,1,100), dns(8,8,8,8),
                gateway(192,168,1,1), subnet(255,255,255,0),
                servidorIP(192,168,1,150);
const int  puertoServidor = 502, idEsclavo = 2;

/* ---------- ASTRONOMÍA ---------- */
const double LATITUD  = 41.56365;
const double LONGITUD = -4.73720;

/* ---------- VIENTO ---------- */
const IPAddress IP_VIENTO (192,168,1,230);
const uint16_t  PUERTO_VIENTO = 502;
const uint8_t   ID_VIENTO     = 1;
const uint16_t  REG_VIENTO    = 99;
const bool      ANEMO_V3      = false;
const float VEL_TORMENTA = 31.0;
const float VEL_ALERTA   = 45.0;
const float HIST_VIENTO  =  3.0;
const uint32_t T_MUESTREO_VIENTO = 1000;
const uint32_t T_TIMEOUT_VIENTO  = 5000;

/* ---------- CONTROL GENERAL ---------- */
const float     TOL_ANGULO = 5.0;
const uint32_t  PERIODO_LOOP_MS = 1000;
const uint32_t  T_MAX_MOV_MS = 20000;
const uint16_t  TIMEOUT_MODBUS  = 1000;
const float     ANG_MIN = -1.5;
const float     ANG_MAX = 85.0;
const float     UMBRAL_CAMBIO   = 5.5;
const uint8_t   MAX_REINTENTOS  = 3;
const uint32_t  ESPERA_REINT_MS = 10UL * 60 * 1000;
const uint32_t TIEMPO_MAX_VALVULA = 20UL * 60 * 1000; // 20 minutos

const unsigned long ZONE_OFFSET = 0UL;   // 2 h × 3600 s

/* ---------- PINES ---------- */
const int RELAY_SUBIR = D1;
const int RELAY_BAJAR = D2;
const int RELAY_IZQ = D3;
const int RELAY_DER = D4;
const int LED_SUBIR   = D7;
const int LED_BAJAR  = D9;
const int LED_ERROR = D8;
const int PIN_SETA    = A1;
const int PIN_ENDSTOP = A0;
const int LED_IZQ   = D10;
const int LED_DER   = D11;
const int LED_ROJO = LEDR;         // LED rojo reset
const int PIN_BREAKER = A7;          // I8 – detecta si el magneto está bajado
const int PIN_ENDSTOP_AZ = A4;  // Asumiendo que usas el pin A4 como I5


/* ---------- VARIABLES GLOBALES ---------- */
volatile EstadoSistema estado = INICIALIZANDO;
volatile EstadoSistema estadoAnterior = INICIALIZANDO;
float anguloObjetivo = 45.0;
float anguloActual = 0.0;
float ultimoValorValido = 0.0;
float anguloInicioMov = 0.0;
uint32_t tUltimoMov  = 0;
uint32_t tUltimoSolar = 0;
uint32_t tProxReint = 0;
uint8_t  numReintentos = 0;
bool     emergenciaActiva = false;
uint8_t reintAz = 0;
uint32_t tProxReintAz = 0;
float offsetInclinometro = 0.0;
const float UMBRAL_ABRUPTO = 30.0;
const uint8_t  ID_SEC345 = 1;
const float   TOL_AZIMUT = 5.0;
double objetivoCenit  = 0.0;
double elevacionSolar = 0.0;
double azimutReal    = 0.0;
double azimutObjetivo= 0.0;
double cenitRealGlobal = 0.0;
const IPAddress IP_SEC345(192,168,1,150);
const uint16_t  PUERTO_SEC345 = 502;
const bool VALVULA_ACTIVA_HIGH = true; // true: la válvula se activa con HIGH
volatile bool breakerEstadoAnterior = HIGH; // suponemos RED OK al inicio
bool     valvulaActiva      = false;
uint32_t tInicioValvulaMs   = 0;          // millis() cuando la encendemos
bool     breakerFallo     = false;    // true mientras A7 esté LOW
uint32_t tProxAvisoMs     = 0;        // para avisos cada 30 s
uint32_t tUltimoNtp   = 0;     // ← NUEVO: marca del último intento NTP
// Buffer para logs
char logBuffer[256];
uint32_t tInicioMovimiento = 0;
uint32_t tLastAviso = 0;
double azimutRealAnguloSolar     = 0.0;  // ① tu lectura de la brújula

/* ---------- OBJETOS ---------- */
EthernetClient        eth;
ModbusTCPClient       modbus(eth);
EthernetUDP           ntpUDP;
NTPClient             ntp(ntpUDP, "hora.roa.es", 0, 60000);
mbed::Watchdog       &perro = mbed::Watchdog::get_instance();
EthernetClient        ethViento;
ModbusTCPClient       modbusViento(ethViento);
EthernetClient        ethBrujula;
ModbusTCPClient       modbusBr(ethBrujula);

/* ========================================================= */
/* ======================   SET-UP   ======================== */
/* ========================================================= */
void setup() {
    Serial.begin(BAUDIOS_SERIAL);
    while (!Serial && millis() < 3000);

    pinMode(RELAY_SUBIR, OUTPUT);
    pinMode(RELAY_BAJAR, OUTPUT);
    pinMode(LED_SUBIR, OUTPUT);
    pinMode(LED_BAJAR, OUTPUT);
    pinMode(LED_ERROR, OUTPUT);
    pinMode(PIN_ENDSTOP, INPUT_PULLUP);
    pinMode(PIN_SETA, INPUT);
    pinMode(RELAY_IZQ, OUTPUT);
    pinMode(RELAY_DER, OUTPUT);
    pinMode(LED_IZQ, OUTPUT);
    pinMode(LED_DER, OUTPUT);
    pinMode(PIN_ENDSTOP_AZ, INPUT_PULLUP);  

    digitalWrite(RELAY_IZQ, LOW);
    digitalWrite(RELAY_DER, LOW);
    digitalWrite(LED_IZQ, LOW);
    digitalWrite(LED_DER, LOW);
    digitalWrite(RELAY_SUBIR, LOW);
    digitalWrite(RELAY_BAJAR, LOW);

    inicializarBreaker();

    /* Ethernet */
    uint32_t t0 = millis();
    while (!inicializarEthernet() && millis() - t0 < 10000) {
        LOG_ACCION("Reintento Ethernet");
        delay(500);
    }
    
    ntp.begin();
    ntp.setTimeOffset(CORR_TIEMPO_SEG);

    /* NTP + primer cálculo solar */
    for (int i = 0; i < 3 && !sincronizarHoraNTP(); i++) delay(1000);
    tUltimoNtp = millis();            // ← añade esta línea
    actualizarAnguloSolar(millis());


    
    /* Modbus */
    t0 = millis();
    while (!inicializarModbus() && millis() - t0 < 5000) {
        LOG_ACCION("Reintento Modbus");
        delay(500);
    }
    
    if (!modbusViento.begin(IP_VIENTO, PUERTO_VIENTO)) {
        LOG_ERROR_("No conecta Modbus viento → ALERTA");
        estadoViento = VIENTO_ALERTA;
    }
    
    /* Brújula SEC345 */
    if (!modbusBr.begin(IP_SEC345, PUERTO_SEC345)) {
        LOG_ERROR_("No conecta Modbus brújula");
    } else {
        delay(1000);
        if (configurarModoPreguntaRespuesta())
            {
                LOG_INFO("Brújula en modo Q&A");
            }
        else
            {
                LOG_ADVERT("No pudo activar Q&A en brújula");
            }
                /* ===== INSERTA DESDE AQUÍ ===== */
        LOG_INFO("Auto-calibrando Sur… (3 s)");
        t0 = millis();                    
        uint16_t ok   = 0;
        double   suma = 0;

        while (millis() - t0 < TIEMPO_CALIB_MS) {
            float az;
            if (leerAzimutSEC345(az)) {   // usa la versión NUEVA de la función
                suma += az;
                ok++;
            }
            perro.kick();
            delay(50);
        }

        if (ok >= 5) {                    // al menos 5 lecturas válidas
            azSurRef         = suma / ok; // promedio
            brujulaCalibrada = true;
            snprintf(logBuffer, sizeof(logBuffer),
                    "Calibración OK – Sur = %.1f°  (muestras=%u)", azSurRef, ok);
            LOG_INFO(logBuffer);
        } else {
            LOG_ADVERT("Calibración brujula FALLIDA – seguirán grados absolutos");
        }
    }

    /* Primer ángulo del inclinómetro */
    LOG_INFO("Esperando primera lectura inclinómetro…");
    t0 = millis();
    while (true) {
        float aux = leerAnguloActual();
        if (!isnan(aux) && aux >= ANG_MIN && aux <= ANG_MAX) {
            anguloActual = ultimoValorValido = anguloInicioMov = aux;
            snprintf(logBuffer, sizeof(logBuffer), "Primer ángulo: %.1f °", aux);
            LOG_INFO(logBuffer);
            break;
        }
        if (millis() - t0 > 5000) {
            LOG_ERROR_("No llega dato Modbus");
            cambiarEstado(ERROR_MODBUS);
            break;
        }
        delay(200);
    }

    if (estado == INICIALIZANDO) {
        cambiarEstado(OPERATIVO);
        snprintf(logBuffer, sizeof(logBuffer), "Sistema operativo v%s", VERSION_FIRMWARE);
        LOG_INFO(logBuffer);
    }
    perro.start(8000);
}

/* =========================================================
   Cálculo solar → objetivo cenit y objetivo azimut (±180°)
   ========================================================= */
void actualizarAnguloSolar(uint32_t now)
{
    static double lastCenit = -1000.0;
    static uint32_t tLastAjuste = 0;


    /* --- 1. Epoch UTC → hora civil local (UTC+2) ---------------------- */
    unsigned long utc   = ntp.getEpochTime() + CORR_TIEMPO_SEG;
    unsigned long civil = utc + ZONE_OFFSET;

    /* --- 2. Astronomía básica ---------------------------------------- */
    double jcn  = calculateJulianCenturyNumber(civil);
    double eqT  = calculateEquationOfTime(jcn);
    double dec  = calculateDeclination(jcn);
    double lst  = calculateLocalSolarTime(civil, eqT, LONGITUD);
    double ha   = calculateHourAngle(lst);

    double elev = calculateSolarElevation(ha, dec, LATITUD);   // +90° = cenit
    elevacionSolar = elev;

    double azN  = calculateSolarAzimuth(ha, dec, elev, LATITUD);   // 0° = Norte, sentido horario

    /* --- 3. Azimut absoluto normalizado 0-360° ----------------------- */
    if (azN < 0.0)        azN += 360.0;
    else if (azN >= 360.0) azN -= 360.0;

    /* --- 4. Siempre usar Sur geométrico → sistema ±180° ------------- */
    double objRel = azN - 180.0;      // 180° es Sur geométrico

    // Normalizar a –180 … +180
    if (objRel >  180.0)  objRel -= 360.0;
    if (objRel <= -180.0) objRel += 360.0;

    // Histéresis de 1° para considerar Sur exacto
    if (fabs(objRel) < 1.0) objRel = 0.0;

    /* --- 5. Añadir tu desfase de calado definido en config --------- */
    objRel += DESFASE_AZIMUT;

    // Y normalizar de nuevo
    if (objRel >  180.0)  objRel -= 360.0;
    if (objRel <= -180.0) objRel += 360.0;

    double ajusteAz = 0.0;
    if      (objRel >  0.0) ajusteAz =  -8.0;
    else if (objRel <  0.0) ajusteAz = 8.0;
    // (si objRel == 0 no ajusta)

    // Asigna ahora el objetivo con ese desfase adicional
    azimutObjetivo = objRel + ajusteAz;
    //zimutObjetivo       = objRel ;
    azimutRealAnguloSolar= objRel;

    /* --- 6. Cálculo de cenit y objetivo con offset ----------------- */
    double cenitReal = 90.0 - elev;
    cenitRealGlobal  = cenitReal;               // sin offset aquí

    double cenitUso  = (cenitReal > CENIT_MAX) ? CENIT_MAX : cenitReal;
    objetivoCenit    = cenitUso + DESFASE_CENIT;

    if (elevacionSolar > 5.0) {
        double nuevoObjetivo = cenitUso + DESFASE_CENIT;
        if (fabs(anguloObjetivo - nuevoObjetivo) > 0.5) {
            anguloObjetivo = nuevoObjetivo;
        }
    } else {
        anguloObjetivo = 0.0;
    }

    /* --- 7. Debug completo en cada invocación --------------------- */
    char debugBuf[256];
    snprintf(debugBuf, sizeof(debugBuf),
        "utc=%lu, civil=%lu, jcn=%.6f, eqT=%.6f, dec=%.6f, lst=%.6f, ha=%.6f, "
        "elev=%.2f, azN=%.2f, objRel=%.2f, cenitReal=%.2f, cenitUso=%.2f, "
        "objetivoCenit=%.2f, anguloObjetivo=%.2f",
        utc, civil, jcn, eqT, dec, lst, ha,
        elev, azN, objRel, cenitReal, cenitUso,
        objetivoCenit, anguloObjetivo
    );
    LOG_MEDIDA(debugBuf);

    /* --- 8. Log cada 5 min o si cambia ≥0,1° ----------------------- */
    if (fabs(cenitRealGlobal - lastCenit) > 0.1 || now - tUltimoSolar > 300000)
    {
        snprintf(logBuffer, sizeof(logBuffer),
                 "Cenit=%.2f° → Obj(C)=%.2f°",
                 cenitReal, anguloObjetivo);
        LOG_MEDIDA(logBuffer);
        lastCenit = cenitRealGlobal;
    }
}

/* =========================================================
   BUCLE PRINCIPAL OPTIMIZADO
   ========================================================= */
void loop() {
    static uint32_t last_cycle = 0;
    uint32_t now = millis();

    // Control estricto del ciclo
    if (now - last_cycle < PERIODO_LOOP_MS) {
        delay(PERIODO_LOOP_MS - (now - last_cycle));
    }
    last_cycle = now;

    perro.kick();  // Alimentar watchdog
    vigilarBreaker();     
        /* ---------- PAUSA GLOBAL POR FALLO BREAKER ---------- */
    if (breakerFallo) {

        /* Aviso cada 30 s del tiempo que queda para desconexión */
        if (millis() >= tProxAvisoMs && valvulaActiva) {
            uint32_t msRest = TIEMPO_MAX_VALVULA - (millis() - tInicioValvulaMs);
            uint32_t min    = msRest / 60000;
            uint32_t seg    = (msRest % 60000) / 1000;
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "Electroválvula ON – quedan %lu min %02lu s",
                     min, seg);
            LOG_ADVERT(buf);
            tProxAvisoMs = millis() + 30000;   // próximo aviso en 30 s
        }

        /* Cuando pasan los 20 min, valvulaActiva ya se puso false dentro de vigilarBreaker();
           sólo seguimos avisando de que el fallo persiste cada 30 s */
        if (!valvulaActiva && millis() >= tProxAvisoMs) {
            LOG_ADVERT("Fallo persiste – válvula OFF, espera subir magneto");
            tProxAvisoMs = millis() + 30000;
        }

        perro.kick();       // alimenta watchdog
        delay(50);          // pequeño respiro
        return;             // <<< salta todo el resto del loop
    }

    /* -------- EMERGENCIA (SETA) -------- */
    if (digitalRead(PIN_SETA) == LOW) {          // ← seta pulsada
        if (!emergenciaActiva) {
            LOG_ERROR_("¡¡EMERGENCIA ACTIVADA!!");
            emergenciaActiva = true;
            cambiarEstado(ERROR_SEGURIDAD);
        }
        digitalWrite(LED_ROJO, HIGH);            // << NUEVO: enciende LED rojo
        digitalWrite(RELAY_SUBIR, LOW);
        digitalWrite(RELAY_BAJAR, LOW);
        digitalWrite(RELAY_IZQ, LOW);
        digitalWrite(RELAY_DER, LOW);
        return;
    } else if (emergenciaActiva) {               // ← seta liberada
        LOG_ACCION("Emergencia desactivada, recalibrando…");
        emergenciaActiva = false;
        offsetInclinometro = 0.0;
        digitalWrite(LED_ROJO, LOW);             // << NUEVO: apaga LED rojo
        reiniciarSistema();
    }

    perro.kick();

    /* -------- VIENTO -------- */
    float v = leerViento();
    actualizarEstadoViento(v);
    aplicarProteccionViento();

    /* 1. actualiza hora NTP cada 5-10 min (si falla, no pasa nada) */
    if (now - tUltimoNtp > 600000) {          // cada 10 min
        sincronizarHoraNTP();                 // ignora el retorno
        tUltimoNtp = now;
    }

    /* 2. calcula posición solar SIEMPRE cada 30 s (o 1 min) */
    if (now - tUltimoSolar > 30000) {
        actualizarAnguloSolar(now);           // usa la hora local ya corregida
        tUltimoSolar = now;
    }

    perro.kick();

    /* -------- Inclinómetro -------- */
    float nuevo = leerAnguloActual();
    anguloActual = 0.7f * anguloActual + 0.3f * nuevo;
    verificarSeguridadMecanica(anguloActual);
    verificarSeguridadAzimut();

    perro.kick();

    /* -------- FSM Cenit -------- */
    switch (estado) {
        case OPERATIVO:
        case ALINEADO: {
            bool subir = false, bajar = false;
            controlarSalidas(subir, bajar);
            if (subir)      cambiarEstado(MOVIENDO_ARRIBA);
            else if (bajar) cambiarEstado(MOVIENDO_ABAJO);
        } break;

        case MOVIENDO_ARRIBA:
        case MOVIENDO_ABAJO:
            if (fabs(anguloActual - anguloInicioMov) < UMBRAL_CAMBIO &&
                now - tUltimoMov > 5000) {
                manejarErrorMovimiento();
            } else if (fabs(anguloActual - anguloObjetivo) <= TOL_ANGULO) {
                cambiarEstado(ALINEADO);
                numReintentos = 0;
            }
            break;

        case REINTENTO_PENDIENTE:
            if (now >= tProxReint) {
                LOG_ACCION("Reintento de movimiento");
                cambiarEstado(OPERATIVO);
            } else {
                if (now - tLastAviso > 30000) {
                    uint32_t ms_restante = tProxReint - now;
                    uint32_t minutos = ms_restante / 60000;
                    uint32_t segundos = (ms_restante % 60000) / 1000;
                    char tiempoBuffer[50];
                    snprintf(tiempoBuffer, sizeof(tiempoBuffer), "Reintento en %lu min %02lu s", minutos, segundos);
                    LOG_ADVERT(tiempoBuffer);
                    tLastAviso = now;
                }
            }
            break;
        case ERROR_MOVIMIENTO:
            manejarPosicionSegura(now);
            break;

        default:
            break;
    }

    perro.kick();

    /* =========================================================
    BUCLE PRINCIPAL OPTIMIZADO (SECTOR BRÚJULA)
    ========================================================= */
    /* -------- LECTURA BRÚJULA -------- */
    static uint32_t ultimoLogBrujula = 0;
    const uint32_t INTERVALO_LOG_BRUJULA = 30000; // 30 segundos

    float nuevaMedidaAz;
    if (leerAzimutSEC345(nuevaMedidaAz)) {
        azimutReal = nuevaMedidaAz;
        
        // Mostrar solo si ha pasado el intervalo y el valor cambió
        static float ultimoAzimut = 0;
        if (millis() - ultimoLogBrujula > INTERVALO_LOG_BRUJULA && 
            fabs(azimutReal - ultimoAzimut) > 0.5) {
            char tempBuffer[100];
           // snprintf(tempBuffer, sizeof(tempBuffer), "🧭 Azimut: %.1f°", azimutReal);
            //LOG_MEDIDA(tempBuffer);
            ultimoLogBrujula = millis();
            ultimoAzimut = azimutReal;
        }
    } else {
        static uint32_t ultimoErrorBrujula = 0;
        const uint32_t INTERVALO_ERROR_BRUJULA = 60000; // 60 segundos
        
        // Registrar error máximo 1 vez por minuto
        if (millis() - ultimoErrorBrujula > INTERVALO_ERROR_BRUJULA) {
            LOG_ADVERT("Error leyendo brújula");
            ultimoErrorBrujula = millis();
        }
        cambiarEstadoAz(AZ_REINTENTO);
    }

    /* -------- FSM Azimut -------- */
    if (estado == ALINEADO) {
        float d = azimutObjetivo - azimutReal;
        if (isnan(azimutReal)) {
            cambiarEstadoAz(AZ_REINTENTO);
        } else if (fabs(d) <= TOL_AZIMUT) {
            cambiarEstadoAz(AZ_ALINEADO);
        } else if (d > TOL_AZIMUT) {
            cambiarEstadoAz(MOV_AZ_IZQ);
        } else if (d < -TOL_AZIMUT) {
            cambiarEstadoAz(MOV_AZ_DER);
        }

        if (estadoAz == AZ_REINTENTO && now >= tProxReintAz) {
            cambiarEstadoAz(AZ_INIT);
        }
    }
    float d = azimutObjetivo - azimutReal;   // diferencia absoluta

    /* --- NORMALIZAR a rango –180 … +180 --- */
    if (d >  180.0f)  d -= 360.0f;
    if (d <= -180.0f) d += 360.0f;

    perro.kick();


    /* =========================================================
    LOG PERIÓDICO OPTIMIZADO (SIN DUPLICADOS)
    ========================================================= */
    /* -------- LOG cada 30 segundos -------- */
    static uint32_t tLog = 0;
    if (now - tLog >= 30000) {
    char mensaje[300];

    const char* lado =                       // NUEVO
          (azimutReal == 0.0f) ? "Sur" :
          (azimutReal  > 0.0f) ? "Oeste" : "Este";
    const char* ladoObj =
      (azimutObjetivo == 0.0f) ? "Sur" :
      (azimutObjetivo  > 0.0f) ? "Oeste" : "Este";

    snprintf(mensaje, sizeof(mensaje),
        "🌡 Cenit=%.1f° | 🎯 ObjC=%.1f° | 📐 CenitReal=%.1f° | 🌞 Elev=%.1f° | "
        "🧭 AzR=%+.1f° (%s) | 🎯 ObjA=%.1f° (%s) | 🔆 AzSolar=%+.1f°  | 💨 Viento=%.1f m/s (%s)",
        anguloActual, 
        anguloObjetivo,
        cenitRealGlobal,
        elevacionSolar,
        azimutReal,          //  <── 1º argumento nuevo
        lado,                //  <── 2º argumento nuevo
        azimutObjetivo,
        ladoObj,
        azimutRealAnguloSolar,
        v, 
        textoEstadoViento(estadoViento));
        
        LOG_MEDIDA(mensaje);
        tLog = now;
    }

    perro.kick();
}


/* =========================================================
   FUNCIONES AUXILIARES
   ========================================================= */

void manejarPosicionSegura(uint32_t now) {
    switch (estadoPosicion) {
        case PS_INACTIVO:
            if (anguloActual > 50) {
                digitalWrite(RELAY_BAJAR, HIGH);
                estadoPosicion = PS_BAJANDO;
                tInicioMovimiento = now;
                LOG_ACCION("Bajando a posicion segura");
            } else if (anguloActual < 40) {
                digitalWrite(RELAY_SUBIR, HIGH);
                estadoPosicion = PS_SUBIENDO;
                tInicioMovimiento = now;
                LOG_ACCION("Subiendo a posicion segura");
            } else {
                estadoPosicion = PS_COMPLETADO;
            }
            break;
            
        case PS_BAJANDO:
            if (anguloActual <= 45 || now - tInicioMovimiento > 30000) {
                digitalWrite(RELAY_BAJAR, LOW);
                estadoPosicion = PS_COMPLETADO;
                LOG_ACCION("Posición segura alcanzada (bajando)");
            }
            break;
            
        case PS_SUBIENDO:
            if (anguloActual >= 45 || now - tInicioMovimiento > 30000) {
                digitalWrite(RELAY_SUBIR, LOW);
                estadoPosicion = PS_COMPLETADO;
                LOG_ACCION("Posición segura alcanzada (subiendo)");
            }
            break;
            
        case PS_COMPLETADO:
            estadoPosicion = PS_INACTIVO;
            cambiarEstado(MODO_ESPERA);
            break;
    }
}

const char* nombreEstado(EstadoSistema e) {
    switch (e) {
        case INICIALIZANDO:      return "INICIALIZANDO";
        case OPERATIVO:          return "OPERATIVO";
        case MOVIENDO_ARRIBA:    return "MOV_ARRIBA";
        case MOVIENDO_ABAJO:     return "MOV_ABAJO";
        case ALINEADO:           return "ALINEADO";
        case ERROR_NTP:          return "ERR_NTP";
        case ERROR_MODBUS:       return "ERR_MODBUS";
        case ERROR_SEGURIDAD:    return "ERR_SEGURIDAD";
        case ERROR_MOVIMIENTO:   return "ERR_MOVIM";
        case REINTENTO_PENDIENTE:return "REINTENTO";
        case MODO_ESPERA:        return "ESPERA";
        default:                 return "DESCONOC";
    }
}

const char* textoEstadoViento(EstadoViento e) {
    switch (e) {
        case VIENTO_OK:       return "OK";
        case VIENTO_TORMENTA: return "TORMENTA";
        case VIENTO_ALERTA:   return "ALERTA";
        default:              return "DESCONOCIDO";
    }
}

void registrarEvento(const char* tipo, const char* mensaje, bool critico) {
    char buffer[256];
    const char* hora = ntp.getFormattedTime().c_str();
    if (!hora || hora[0] == '\0') hora = "00:00:00";
    
    snprintf(buffer, sizeof(buffer), "[%s] %s: %s", hora, tipo, mensaje);
    Serial.println(buffer);

    if (critico) {
        digitalWrite(LED_ERROR, HIGH);
        delay(120);
        digitalWrite(LED_ERROR, LOW);
    }
}

bool configurarModoPreguntaRespuesta() {
    if (!modbusBr.beginTransmission(HOLDING_REGISTERS, 0x0000, sizeof(SET_QA_MODE)))
        return false;
    for (auto b : SET_QA_MODE) modbusBr.write(b);
    return (modbusBr.endTransmission() == 0);
}

/* =========================================================
   FUNCIÓN DE LECTURA DE INCLINÓMETRO SILENCIOSA
   ========================================================= */
float leerAnguloActual() {
    static uint32_t tErr = 0; 
    static bool first = true;
    const uint32_t ESPERA = 30000; // 30 segundos entre reintentos
    static uint32_t ultimoLog = 0;

    if (!modbus.connected()) {
        if (millis() - tErr > ESPERA) {
            // Reconexión silenciosa
            modbus.begin(servidorIP, puertoServidor);
            tErr = millis();
        }
        return ultimoValorValido;
    }

    if (modbus.requestFrom(idEsclavo, HOLDING_REGISTERS, 0x0001, 1)) {
        int16_t raw = modbus.read();
        float ang = (raw - 20000) / 100.0 - offsetInclinometro;

        if (isnan(ang)) {
            return ultimoValorValido;
        }

        /* salto brusco */
        if (!first && fabs(ang - ultimoValorValido) > 10.0) {
            if (fabs(ang - ultimoValorValido) > UMBRAL_ABRUPTO) {
                offsetInclinometro += ang - ultimoValorValido;
                return ultimoValorValido;
            }
        }
        first = false;

        if (ang >= ANG_MIN && ang <= ANG_MAX) {
            ultimoValorValido = ang;
            if (estado == ERROR_MODBUS) cambiarEstado(OPERATIVO);
            return ang;
        }
    }

    if (millis() - tErr > ESPERA) {
        tErr = millis();
    }
    return ultimoValorValido;
}

/* ---- Ethernet ---- */
bool inicializarEthernet() {
    LOG_ACCION("Inicializando Ethernet");
    Ethernet.begin(mac, ip, dns, gateway, subnet);
    delay(1000);
    if (Ethernet.linkStatus() == LinkOFF) { 
        LOG_ERROR_("Sin cable"); 
        return false; 
    }
    if (Ethernet.localIP() == IPAddress(0,0,0,0)) { 
        LOG_ERROR_("IP nula"); 
        return false; 
    }
    snprintf(logBuffer, sizeof(logBuffer), "IP asignada %s", Ethernet.localIP().toString().c_str());
    LOG_INFO(logBuffer);
    return true;
}
/* =========================================================
   FUNCIONES AUXILIARES (continuación)
   ========================================================= */

/* ---- Modbus principal ---- */
bool inicializarModbus() {
    LOG_ACCION("Conectando Modbus principal…");
    if (!modbus.begin(servidorIP, puertoServidor)) {
        LOG_ERROR_("No conecta Modbus principal");
        return false;
    }
    modbus.setTimeout(TIMEOUT_MODBUS);
    LOG_INFO("Modbus principal OK");
    return true;
}

/* ---- Sincronización NTP ---- */
bool sincronizarHoraNTP() {
    if (ntp.update()) {
        snprintf(logBuffer, sizeof(logBuffer), "Hora UTC: %s", ntp.getFormattedTime().c_str());
        LOG_INFO(logBuffer);
        return true;
    }
    snprintf(logBuffer, sizeof(logBuffer), "Fallo NTP. Epoch=%lu", ntp.getEpochTime());
    LOG_ERROR_(logBuffer);
    return false;
}

/* -------- Control de salidas -------- */
void controlarSalidas(bool &subir, bool &bajar) {
    float diferencia = anguloObjetivo - anguloActual;
    float tolerancia = constrain(fabs(diferencia) / 4.0, 0.5, TOL_ANGULO);

    subir =  (diferencia >  tolerancia);
    bajar =  (diferencia < -tolerancia);

    /* Bloqueo si el end-stop superior está activo */
    if (subir && digitalRead(PIN_ENDSTOP) == HIGH) {
        subir = false;
        LOG_ADVERT("End-stop superior activo: bloqueada orden de SUBIR");
    }
}

/* ---------- Seguridad mecánica ---------- */
void verificarSeguridadMecanica(float angulo) {
    /* End-stop hardware */
    if (digitalRead(PIN_ENDSTOP) == HIGH) {
        if (estado == MOVIENDO_ARRIBA) {
            LOG_ERROR_("End-stop alcanzado → parada inmediata");
            cambiarEstado(ERROR_SEGURIDAD);
        }
        return;
    }

    if (angulo <= ANG_MIN && estado == MOVIENDO_ABAJO) {
        LOG_ERROR_("Límite inferior alcanzado");
        cambiarEstado(ERROR_SEGURIDAD);
    }
    if (angulo >= ANG_MAX && estado == MOVIENDO_ARRIBA) {
        LOG_ERROR_("Límite superior alcanzado");
        cambiarEstado(ERROR_SEGURIDAD);
    }

    /* Reset dinámico del timeout por avance >=0,5° */
    static float ultimoAngTimeout = angulo;
    if (estado == MOVIENDO_ARRIBA || estado == MOVIENDO_ABAJO) {
        if (fabs(angulo - ultimoAngTimeout) >= 0.5) {
            tUltimoMov        = millis();
            ultimoAngTimeout  = angulo;
        }
    }

    /* Timeout de movimiento */
    if ((estado == MOVIENDO_ARRIBA || estado == MOVIENDO_ABAJO) &&
        (millis() - tUltimoMov > T_MAX_MOV_MS)) {
        manejarErrorMovimiento();
    }
}

void verificarSeguridadAzimut() {
    if (digitalRead(PIN_ENDSTOP_AZ) == LOW) {  // Fin de carrera activado
        if (estadoAz == MOV_AZ_IZQ || estadoAz == MOV_AZ_DER) {
            LOG_ADVERT("¡Fin de carrera azimut activado! Deteniendo movimiento");
            digitalWrite(RELAY_IZQ, LOW);
            digitalWrite(RELAY_DER, LOW);
            digitalWrite(LED_IZQ, LOW);
            digitalWrite(LED_DER, LOW);
            cambiarEstadoAz(AZ_ERROR_MOV);
        }
    }
}
/* ---------- Gestión de fallos de movimiento ---------- */
void manejarErrorMovimiento() {
    numReintentos++;
    if (numReintentos >= MAX_REINTENTOS) {
        snprintf(logBuffer, sizeof(logBuffer), 
            "Fallo crítico tras %u intentos", numReintentos);
        LOG_ERROR_(logBuffer);
        cambiarEstado(ERROR_MOVIMIENTO);
    } else {
        snprintf(logBuffer, sizeof(logBuffer),
            "Movimiento estancado. Reintento %u/%u",
            numReintentos, MAX_REINTENTOS);
        LOG_ADVERT(logBuffer);
        cambiarEstado(REINTENTO_PENDIENTE);
        tProxReint = millis() + ESPERA_REINT_MS;
    }
}

/* ---------- Reinicio suave del sistema ---------- */
void reiniciarSistema() {
    LOG_ACCION("Reiniciando subsistemas…");
    inicializarEthernet();
    sincronizarHoraNTP();
    inicializarModbus();
    numReintentos   = 0;
    tProxReint      = 0;
    anguloActual    = leerAnguloActual();
    cambiarEstado(OPERATIVO);
    LOG_INFO("Reinicio completo");
}

/* ---------- Cambio de estado (con LEDs y relés) ---------- */
void cambiarEstado(EstadoSistema nuevo) {
    if (estado == nuevo) return;

    snprintf(logBuffer, sizeof(logBuffer), 
        "%s → %s", nombreEstado(estado), nombreEstado(nuevo));
    LOG_ESTADO(logBuffer);

    /* Apaga todo antes de decidir */
    digitalWrite(LED_SUBIR, LOW);
    digitalWrite(LED_BAJAR, LOW);
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(RELAY_SUBIR, LOW);
    digitalWrite(RELAY_BAJAR, LOW);

    if (nuevo == MOVIENDO_ARRIBA) {
        if (digitalRead(PIN_ENDSTOP) == HIGH) {
            LOG_ERROR_("Intento de SUBIR con end-stop activo");
            return;
        }
        digitalWrite(LED_SUBIR, HIGH);
        digitalWrite(RELAY_SUBIR, HIGH);
    }
    else if (nuevo == MOVIENDO_ABAJO) {
        digitalWrite(LED_BAJAR, HIGH);
        digitalWrite(RELAY_BAJAR, HIGH);
    }
    else if (nuevo == ENDSTOP_VERTICAL ||
             nuevo == ERROR_NTP        ||
             nuevo == ERROR_MODBUS     ||
             nuevo == ERROR_SEGURIDAD  ||
             nuevo == ERROR_MOVIMIENTO ||
             nuevo == REINTENTO_PENDIENTE) {
        digitalWrite(LED_ERROR, HIGH);
    }

    if (nuevo == MOVIENDO_ARRIBA || nuevo == MOVIENDO_ABAJO) {
        anguloInicioMov = anguloActual;
    }

    estadoAnterior = estado;
    estado         = nuevo;
    tUltimoMov     = millis();
}

/* ---------- FSM Azimut ---------- */
void cambiarEstadoAz(EstadoAzimut nuevo) {
    if (estadoAz == nuevo) return;

        if ((nuevo == MOV_AZ_IZQ || nuevo == MOV_AZ_DER) && digitalRead(PIN_ENDSTOP_AZ) == LOW) {
        LOG_ADVERT("Intento de movimiento azimut con fin de carrera activado");
        return; // Sale sin cambiar el estado actual
    }

    snprintf(logBuffer, sizeof(logBuffer), 
        "Azimut: %d → %d", estadoAz, nuevo);
    LOG_ESTADO(logBuffer);
    
    // Apagar salidas anteriores
    digitalWrite(RELAY_IZQ, LOW); 
    digitalWrite(LED_IZQ, LOW);
    digitalWrite(RELAY_DER, LOW); 
    digitalWrite(LED_DER, LOW);
    
    switch (nuevo) {
        case MOV_AZ_IZQ:
            digitalWrite(LED_IZQ, HIGH);
            digitalWrite(RELAY_IZQ, HIGH);
            break;
        case MOV_AZ_DER:
            digitalWrite(LED_DER, HIGH);
            digitalWrite(RELAY_DER, HIGH);
            break;
        case AZ_ALINEADO:
            reintAz = 0;
            break;
        case AZ_REINTENTO:
            if (reintAz >= MAX_REINTENTOS) {
                cambiarEstadoAz(AZ_ERROR_MOV);
            } else {
                reintAz++;
                tProxReintAz = millis() + ESPERA_REINT_MS;
            }
            break;
        case AZ_ERROR_MOV:
            LOG_ERROR_("Error crítico en movimiento azimut");
            break;
        default:
            break;
    }
    estadoAz = nuevo;
}

/* =========================================================
   FUNCIÓN MEJORADA PARA LECTURA DE BRÚJULA (MENOS INTRUSIVA)
   ========================================================= */
/* =========================================================
   LEE SEC-345  —  normaliza a ±180° y reconecta si falla
   ========================================================= */
bool leerAzimutSEC345(float &azOut)
{
    const uint16_t REG      = 0x03;   // registro de azimut
    const uint8_t  NREG     = 2;      // 2 words = 3 bytes útiles
    const uint32_t T_RETRY  = 30000;  // 30 s entre intentos de socket
    const uint8_t  MAX_FALL = 5;      // re-abrir socket tras 5 fallos

    static uint32_t tSock  = 0;       // último intento de begin()
    static uint8_t  fallos = 0;       // fallos de lectura seguidos

    /* ---------- 1. Revisión del socket ---------- */
    if (!modbusBr.connected() && millis() - tSock > T_RETRY) {
        modbusBr.stop();
        delay(50);
        modbusBr.begin(IP_SEC345, PUERTO_SEC345);
        tSock = millis();
    }

    /* ---------- 2. Petición Modbus ---------- */
    if (!modbusBr.requestFrom(idSEC345, HOLDING_REGISTERS, REG, NREG)) {
        /* fallo: cuenta y, si excede, reinicia socket ya */
        if (++fallos >= MAX_FALL) {
            modbusBr.stop();
            delay(50);
            modbusBr.begin(IP_SEC345, PUERTO_SEC345);
            fallos = 0;
            tSock  = millis();
        }
        return false;
    }

    /* ---------- 3. Decodifica trama ---------- */
    uint16_t r1 = modbusBr.read();
    uint16_t r2 = modbusBr.read();

    uint8_t b0 = highByte(r1);
    uint8_t b1 = lowByte(r1);
    uint8_t b2 = highByte(r2);

    float azAbs = decodificarAzimut(b0, b1, b2);   // 0-360°

    /* ---------- 4. Conversión a ±180° relativo al Sur ---------- */
    if (brujulaCalibrada && !isnan(azSurRef)) {
        float rel = azAbs - azSurRef;              // diferencia bruta
        if (rel >  180.0f)  rel -= 360.0f;         // normaliza
        if (rel <= -180.0f) rel += 360.0f;
        if (fabs(rel) < 1.0f)  rel = 0.0f;         // histéresis ±1°
        azOut = rel;
    } else {
        azOut = azAbs;                             // sin calibrar
    }

    fallos = 0;                                    // lectura OK
    return true;
}


/* =========================================================
   FUNCIÓN DE DECODIFICACIÓN MEJORADA
   ========================================================= */
float decodificarAzimut(uint8_t b0, uint8_t b1, uint8_t b2)
{
    bool neg      = b0 & 0x80;                // bit 7 = signo
    int centenas  = (b0 >> 4) & 0x07;
    int decenas   =  b0 & 0x0F;
    int unidades  =  b1 >> 4;
    int decimales = (b1 & 0x0F) * 10 + (b2 >> 4);

    float ang = (centenas*100 + decenas*10 + unidades) + decimales / 100.0;
    if (neg) ang = -ang;
    if (ang < 0) ang += 360.0;
    return fmod(ang, 360.0f);                 // 0-360 geométrico
}


/* ---------- Función auxiliar para lectura Modbus ---------- */
int16_t leerRegistroModbus(ModbusTCPClient& client, uint8_t id, uint16_t reg, uint16_t timeout = 1000) {
    uint32_t start = millis();
    while (millis() - start < timeout) {
        if (client.requestFrom(id, HOLDING_REGISTERS, reg, 1)) {
            return client.read();
        }
        delay(50);
    }
    return INT16_MIN; // Valor de error
}

/* ---------- Lectura de viento optimizada ---------- */
float leerViento() {
    static uint32_t t0 = 0;
    static float    last = NAN;

    uint32_t now = millis();
    if (now - t0 < T_MUESTREO_VIENTO) return last;
    t0 = now;

    int16_t raw = leerRegistroModbus(modbusViento, ID_VIENTO, REG_VIENTO, T_TIMEOUT_VIENTO);
    if (raw == INT16_MIN) {
        LOG_ADVERT("Timeout Modbus viento");
        return last;
    }
    
    last = (raw / 10.0) * (ANEMO_V3 ? 0.87 : 1.0);
    return last;
}



/* ---------- Actualización estado viento ---------- */
void actualizarEstadoViento(float v) {
    static EstadoViento anterior = VIENTO_OK;
    
    switch (estadoViento) {
        case VIENTO_OK:
            if (v > VEL_TORMENTA) estadoViento = VIENTO_TORMENTA;
            break;
        case VIENTO_TORMENTA:
            if (v > VEL_ALERTA) estadoViento = VIENTO_ALERTA;
            else if (v < VEL_TORMENTA - HIST_VIENTO) estadoViento = VIENTO_OK;
            break;
        case VIENTO_ALERTA:
            if (v < VEL_ALERTA - HIST_VIENTO) estadoViento = VIENTO_TORMENTA;
            break;
    }
    
    if (estadoViento != anterior) {
        snprintf(logBuffer, sizeof(logBuffer), 
            "→ Estado viento ahora: %s", textoEstadoViento(estadoViento));
        LOG_ADVERT(logBuffer);
        anterior = estadoViento;
    }
}

/* ---------- Protección contra viento ---------- */
void aplicarProteccionViento() {
    if (estadoViento == VIENTO_ALERTA) {
        if (anguloObjetivo != 0.0) {
            LOG_ADVERT("Viento >45 m/s → posición horizontal");
            anguloObjetivo = 0.0;
        }
    } else if (estadoViento == VIENTO_TORMENTA) {
        float nuevo = max(anguloObjetivo / 2.0, 0.0);
        if (fabs(nuevo - anguloObjetivo) > 1.0) {
            LOG_ADVERT("Viento 31-45 m/s → objetivo a la mitad");
            anguloObjetivo = nuevo;
        }
    }
    
}
/* =========================================================
   GESTIÓN DEL MAGNETOTÉRMICO
   ========================================================= */
void inicializarBreaker() {
    pinMode(PIN_BREAKER, INPUT_PULLUP);
    pinMode(LED_BAJAR, OUTPUT);     // tu LED auxiliar libre
    pinMode(LED_ROJO, OUTPUT);    // LED rojo sobre botón RESET

    // Estado seguro (RED OK)
    digitalWrite(LED_BAJAR, LOW);
    digitalWrite(LED_ROJO, LOW);
    digitalWrite(RELAY_BAJAR, VALVULA_ACTIVA_HIGH ? LOW : HIGH);

    LOG_INFO("Monitor magnetotérmico listo");
}

/* ======= vigilarBreaker() ======= */
void vigilarBreaker() {
    bool estadoActual = digitalRead(PIN_BREAKER);   // HIGH = red OK
    static bool breakerPrevio = HIGH;               // evita globals extra

    /* --- Flanco de cambio --- */
    if (estadoActual != breakerPrevio) {
        breakerPrevio = estadoActual;

        if (estadoActual == LOW) {                  // 🔴 FALLO
            LOG_ADVERT("⚡ Magnetotérmico bajado – válvula ON (20 min máx)");
            digitalWrite(LED_BAJAR, HIGH);
            digitalWrite(LED_ROJO, HIGH);
            digitalWrite(RELAY_BAJAR, VALVULA_ACTIVA_HIGH ? HIGH : LOW);

            breakerFallo     = true;                // <── PAUSA activa
            valvulaActiva    = true;
            tInicioValvulaMs = millis();
            tProxAvisoMs     = millis();            // primer aviso inmediato
        } else {                                    // 🟢 Red OK
            LOG_INFO("⚡ Red restablecida – válvula OFF");
            digitalWrite(LED_BAJAR, LOW);
            digitalWrite(LED_ROJO, LOW);
            digitalWrite(RELAY_BAJAR, VALVULA_ACTIVA_HIGH ? LOW : HIGH);

            breakerFallo  = false;                  // <── PAUSA desactiva
            valvulaActiva = false;
        }
    }

    /* --- Timeout de 20 min mientras sigue el fallo --- */
    if (estadoActual == LOW && valvulaActiva) {
        if (millis() - tInicioValvulaMs >= TIEMPO_MAX_VALVULA) {
            LOG_ADVERT("⏱ 20 min alcanzados – válvula OFF (seguirá el LED en ROJO)");
            digitalWrite(RELAY_BAJAR, VALVULA_ACTIVA_HIGH ? LOW : HIGH);
            valvulaActiva = false;
            /* mantenemos LED_ROJO encendido para indicar que el fallo persiste */
        }
    }
}