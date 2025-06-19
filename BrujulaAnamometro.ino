/* =========================================================
   INCLINÓMETRO + MODBUS + NTP + CONTROL MOTORES – v2.4.2
   Arduino Opta (Mbed-OS core)
   ========================================================= */

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <NTPClient.h>
#include <ArduinoModbus.h>
#include <SolarCalculator.h>
#include "mbed.h"

/* ---------- ENUMS ---------- */
enum SistemaEstado {
  INICIALIZANDO, OPERATIVO, MOVIENDO_ARRIBA, MOVIENDO_ABAJO, ALINEADO,
  ERROR_NTP, ERROR_MODBUS, ERROR_SEGURIDAD, ERROR_MOVIMIENTO,
  REINTENTO_PENDIENTE, MODO_ESPERA,ENDSTOP_VERTICAL
};

/* ---------- PROTOTIPOS ---------- */
void registrarEvento(const String&, bool critico=false);
bool inicializarEthernet();
bool inicializarModbus();
bool sincronizarHoraNTP();
float leerAnguloActual();
void controlarSalidas(bool&, bool&);
void actualizarAnguloSolar();
void verificarSeguridadMecanica(float);
void cambiarEstado(SistemaEstado);
String obtenerNombreEstado(SistemaEstado);
void manejarErrorMovimiento();
void reiniciarSistema();
void forzarPosicionSegura();

/* ---------- CONFIG ---------- */
#define FIRMWARE_VERSION "2.4.2"
#define SERIAL_BAUD_RATE 115200

#define LOG_INFO(m)     registrarEvento("INFO: "+String(m))
#define LOG_DECISION(m) registrarEvento("DECISION: "+String(m))
#define LOG_ACTION(m)   registrarEvento("ACTION: "+String(m))
#define LOG_WARNING(m)  registrarEvento("WARNING: "+String(m))
#define LOG_ERROR(m)    registrarEvento("ERROR: "+String(m),true)

/* --- Red --- */
uint8_t mac[]={0x00,0x1A,0xB6,0x03,0x2D,0xA1};
const IPAddress ip(192,168,1,100), dns(8,8,8,8),
                gateway(192,168,1,1), subnet(255,255,255,0),
                serverIP(192,168,1,150);
const int serverPort=502, slaveID=2;

/* --- Astronomía --- */
const double LATITUD=40.4168, LONGITUD=-3.7038, ALTITUD=650;

/* --- Control --- */
const float TOLERANCIA_ANGULO=5.0;
const uint32_t INTERVALO_LECTURA=1000, MAX_TIEMPO_MOVIMIENTO=15000;
const uint16_t TIMEOUT_MODBUS=1000;
const float ANGULO_MINIMO=-0.5, ANGULO_MAXIMO=85.0;
const float VELOCIDAD_MOVIMIENTO_GRADOS_SEG=1.5;
const float UMBRAL_CAMBIO_ANGULO=5.5;
const uint8_t MAX_REINTENTOS_MOVIMIENTO=3;
const uint32_t TIEMPO_ESPERA_REINTENTO=10UL*60*1000;

/* --- Pines --- */
const int RELAY_SUBIR=D1, RELAY_BAJAR=D2;
const int LED_SUBIR=D7, LED_BAJAR=D9, LED_ERROR=D8;
/* >>> NUEVO pin del micro-switch superior (LOW = pulsado) */
const int ENDSTOP_MAX_PIN = A0;
// REEMPLAZO CORRECTO para Opta (core 4.3.1):
const int BOTON_EMERGENCIA = BTN_USER;        // botón “USER” del Opta

/* ---------- Variables ---------- */
volatile SistemaEstado estado=INICIALIZANDO, estadoAnterior=INICIALIZANDO;
float anguloObjetivo=45.0, anguloActual=0.0, ultimoValorValido=0.0,
      anguloInicioMovimiento=0.0, ultimoAnguloLeido=0.0;
uint32_t ultimoMovimientoTiempo=0, ultimoCiclo=0,
         ultimaActualizacionSolar=0, tiempoProximoReintento=0;
uint8_t  contadorReintentos=0;
bool emergenciaActivada=false;

/* ---------- Objetos ---------- */
EthernetClient  ethClient;
ModbusTCPClient modbusClient(ethClient);
EthernetUDP     ntpUDP;
// Cambia esta línea (en la declaración de timeClient):
NTPClient timeClient(ntpUDP, "es.pool.ntp.org", 7200, 60000); // Usa servidor español + offset fijo
mbed::Watchdog &wd=mbed::Watchdog::get_instance();

/* ========================================================= */
void setup(){
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial && millis()<5000);

  pinMode(RELAY_SUBIR,OUTPUT); pinMode(RELAY_BAJAR,OUTPUT);
  pinMode(LED_SUBIR,OUTPUT);   pinMode(LED_BAJAR,OUTPUT);
  pinMode(LED_ERROR,OUTPUT);   pinMode(BOTON_EMERGENCIA,INPUT_PULLUP);
  pinMode(ENDSTOP_MAX_PIN, INPUT_PULLUP);   // final de carrera con pull-up
  digitalWrite(RELAY_SUBIR,LOW); digitalWrite(RELAY_BAJAR,LOW);
  digitalWrite(LED_SUBIR,LOW);   digitalWrite(LED_BAJAR,LOW);
  digitalWrite(LED_ERROR,LOW);

  wd.start(8000);

  /* Ethernet */
  uint32_t t=millis();
  while(!inicializarEthernet() && millis()-t<10000){LOG_ACTION("Reintento Ethernet"); delay(500);}
  /* NTP */
  for(int i=0;i<3;i++){ if(sincronizarHoraNTP()) break; if(i==2) cambiarEstado(ERROR_NTP); delay(1000);}
  /* Modbus */
  t=millis();
  while(!inicializarModbus() && millis()-t<5000){LOG_ACTION("Reintento Modbus"); delay(500);}

  /* Primer ángulo */
  LOG_INFO("Esperando primera lectura inclinómetro…");
  uint32_t t0=millis();
  while(true){
    float aux=leerAnguloActual();
    if(!isnan(aux)&&aux>=ANGULO_MINIMO&&aux<=ANGULO_MAXIMO){
      anguloActual=ultimoValorValido=ultimoAnguloLeido=anguloInicioMovimiento=aux;
      LOG_INFO("Primer ángulo: "+String(aux,1)+"°"); break;}
    if(millis()-t0>5000){LOG_ERROR("No llega dato Modbus"); cambiarEstado(ERROR_MODBUS); break;}
    delay(200);
  }

  if(estado==INICIALIZANDO){cambiarEstado(OPERATIVO); LOG_INFO("Sistema operativo v" FIRMWARE_VERSION);}
}



/* ========================================================= */
  bool emergenciaLeidaLow()
  {
    const uint16_t DEBOUNCE_MS = 40;
    if (digitalRead(BOTON_EMERGENCIA) == LOW) {
      delay(DEBOUNCE_MS);
      return digitalRead(BOTON_EMERGENCIA) == LOW;   // confirma LOW
    }
    return false;
  }
/* ========================================================= */
void loop(){
  wd.kick();
  uint32_t inicio=millis();

  /* --- Emergencia --- */
  if (emergenciaLeidaLow()) {
      if (!emergenciaActivada) {
          LOG_ERROR("¡EMERGENCIA ACTIVADA!");
          emergenciaActivada = true;
          cambiarEstado(ERROR_SEGURIDAD);
      }
      digitalWrite(RELAY_SUBIR, LOW);
      digitalWrite(RELAY_BAJAR, LOW);
      return;
  } else if (emergenciaActivada) {
      LOG_ACTION("Emergencia desactivada. Reiniciando sistema…");
      emergenciaActivada = false;
      reiniciarSistema();
  }

  /* --- NTP cada 5 min --- */
  if(millis()-ultimaActualizacionSolar>300000){
    if(sincronizarHoraNTP()){actualizarAnguloSolar(); ultimaActualizacionSolar=millis();}
  }

  /* --- Sensor --- */
  float nuevo=leerAnguloActual();
  anguloActual=0.7f*anguloActual+0.3f*nuevo;
  verificarSeguridadMecanica(anguloActual);

  /* --- FSM --- */
  switch(estado){
    case OPERATIVO: case ALINEADO:{
      bool up=false,down=false; controlarSalidas(up,down);
      if(up) cambiarEstado(MOVIENDO_ARRIBA); else if(down) cambiarEstado(MOVIENDO_ABAJO);} break;

    case MOVIENDO_ARRIBA: case MOVIENDO_ABAJO:
      if(abs(anguloActual-anguloInicioMovimiento)<UMBRAL_CAMBIO_ANGULO &&
         millis()-ultimoMovimientoTiempo>5000) manejarErrorMovimiento();
      else if(abs(anguloActual-anguloObjetivo)<=TOLERANCIA_ANGULO){
        cambiarEstado(ALINEADO); contadorReintentos=0;} break;

    case REINTENTO_PENDIENTE:
      if(millis()>=tiempoProximoReintento){LOG_ACTION("Reintentando movimiento"); cambiarEstado(OPERATIVO);} break;

    case MODO_ESPERA: break;
    default: break;
  }

  /* --- Log 5 s --- */
  static uint32_t ultimoLog=0;
  if(millis()-ultimoLog>=5000){
    String txt="Estado:"+obtenerNombreEstado(estado)+" | Ang:"+String(anguloActual,1)+"° | Obj:"+String(anguloObjetivo,1);
    if(estado==REINTENTO_PENDIENTE) txt+=" | Reintento "+String((tiempoProximoReintento-millis())/60000)+"m";
    LOG_INFO(txt); ultimoLog=millis();
  }

  uint32_t dur=millis()-inicio; if(dur<INTERVALO_LECTURA) delay(INTERVALO_LECTURA-dur); ultimoCiclo=millis();
}

/* =========================================================
   FUNCIONES
   ========================================================= */

float leerAnguloActual(){
  static uint32_t ultimoErr=0; static bool first=true; const uint32_t ESPERA=5000;

  if(!modbusClient.connected()){
    if(millis()-ultimoErr>ESPERA){
      LOG_ACTION("Reconectando Modbus…");
      if(!modbusClient.begin(serverIP,serverPort)){
        LOG_ERROR("Fallo persistente Modbus"); cambiarEstado(ERROR_MODBUS); ultimoErr=millis();
        return ultimoValorValido;
      }
    }
  }

  if(modbusClient.requestFrom(slaveID,HOLDING_REGISTERS,0x0001,1)){
    int16_t raw=modbusClient.read();
    float ang=(raw-20000)/100.0;
    if(isnan(ang)){LOG_ERROR("Dato NaN"); return ultimoValorValido;}

    if(!first && abs(ang-ultimoValorValido)>10.0){
      LOG_WARNING("Cambio abrupto "+String(ang-ultimoValorValido,1));
      if(abs(ang-ultimoValorValido)>30.0) return ultimoValorValido;
    }
    first=false;

    if(ang>=ANGULO_MINIMO && ang<=ANGULO_MAXIMO){
      ultimoValorValido=ang;
      if(estado==ERROR_MODBUS) cambiarEstado(OPERATIVO);
      return ang;
    }else LOG_ERROR("Ángulo fuera rango: "+String(ang,1));
  }

  if(millis()-ultimoErr>ESPERA){LOG_ERROR("Fallo lectura Modbus"); cambiarEstado(ERROR_MODBUS); ultimoErr=millis();}
  return ultimoValorValido;
}

/* ---------- Ethernet ---------- */
bool inicializarEthernet(){
  LOG_ACTION("Init Ethernet"); Ethernet.begin(mac,ip,dns,gateway,subnet); delay(1000);
  if(Ethernet.linkStatus()==LinkOFF){LOG_ERROR("Sin cable"); return false;}
  if(Ethernet.localIP()==IPAddress(0,0,0,0)){LOG_ERROR("IP nula"); return false;}
  LOG_INFO("IP "+Ethernet.localIP().toString()); return true;
}

/* ---------- NTP ---------- */
bool sincronizarHoraNTP() {
    timeClient.begin();
    if(timeClient.update()) {
        LOG_INFO("Hora española (UTC+1): " + timeClient.getFormattedTime());
        return true;
    }
    LOG_ERROR("Fallo NTP");
    return false;
}

/* ---------- Modbus ---------- */
bool inicializarModbus(){ LOG_ACTION("Conectando Modbus…");
  if(!modbusClient.begin(serverIP,serverPort)){LOG_ERROR("No conecta Modbus"); return false;}
  modbusClient.setTimeout(TIMEOUT_MODBUS); LOG_INFO("Modbus OK"); return true;}

/* ---------- Solar ---------- */
void actualizarAnguloSolar(){ time_t e=timeClient.getEpochTime(); double az,el;
  calcHorizontalCoordinates(e,LATITUD,LONGITUD,az,el); LOG_DECISION("Elev "+String(el,1)+"° Az "+String(az,1));
  if(el>=-10&&el<=95&&fabs(anguloObjetivo-el)>0.5){LOG_ACTION("Nuevo obj "+String(el,1)); anguloObjetivo=el;}
}

/* ---------- Salidas ---------- */
void controlarSalidas(bool &up, bool &down)
{
    float diff = anguloObjetivo - anguloActual;
    float tol  = constrain(abs(diff) / 4.0, 0.5, TOLERANCIA_ANGULO);

    up   = (diff >  tol);
    down = (diff < -tol);

    /* ---------- NUEVO: proteger contra subir con end-stop ---------- */
    if (up && endstopSuperiorActivo()) {
        up = false;                                         // anula la orden
        LOG_WARNING("End-stop Vertical activado; no se permite SUBIR");
    }
}
inline bool endstopSuperiorActivo()
{
     return digitalRead(ENDSTOP_MAX_PIN) == HIGH;
}
/* ---------- Seguridad mecánica ---------- */
void verificarSeguridadMecanica(float ang)
  {
      /* ----- End-stop superior (hardware) ----- */
        if (endstopSuperiorActivo()) {
            if (estado == MOVIENDO_ARRIBA) {              // iba subiendo
                LOG_ERROR("End-stop alcanzado: parada inmediata");
                cambiarEstado(ERROR_SEGURIDAD);           // o ALINEADO si prefieres
            }
            // si el ángulo todavía no supera ANGULO_MAXIMO evitamos que vuelva a SUBIR
            return;                                       // se sale: nada más que chequear
        }
      /* --- límites --- */
      if (ang <= ANGULO_MINIMO && estado == MOVIENDO_ABAJO) {
          LOG_ERROR("Límite inferior"); cambiarEstado(ERROR_SEGURIDAD);
      }
      if (ang >= ANGULO_MAXIMO && estado == MOVIENDO_ARRIBA) {
          LOG_ERROR("Límite superior"); cambiarEstado(ERROR_SEGURIDAD);
      }

      /* --- reset dinámico del timeout --- */
      static float ultimoAnguloTimeout = ang;
      if (estado == MOVIENDO_ARRIBA || estado == MOVIENDO_ABAJO) {
          if (fabs(ang - ultimoAnguloTimeout) >= 0.5) {     // avance ≥0.5°
              ultimoMovimientoTiempo = millis();            // ← RESET
              ultimoAnguloTimeout    = ang;
          }
      }

      /* --- timeout --- */
      if ((estado == MOVIENDO_ARRIBA || estado == MOVIENDO_ABAJO) &&
          (millis() - ultimoMovimientoTiempo > MAX_TIEMPO_MOVIMIENTO)) {
          LOG_ERROR("Timeout de movimiento"); manejarErrorMovimiento();
      }
  }

/* ---------- FSM helpers ---------- */
void manejarErrorMovimiento(){
  contadorReintentos++;
  if(contadorReintentos>=MAX_REINTENTOS_MOVIMIENTO){
    LOG_ERROR("Fallo crítico tras "+String(contadorReintentos)+" intentos"); cambiarEstado(ERROR_MOVIMIENTO);
    forzarPosicionSegura(); cambiarEstado(MODO_ESPERA);
  }else{
    LOG_WARNING("Movimiento estancado. Reintento "+String(contadorReintentos)+"/"+String(MAX_REINTENTOS_MOVIMIENTO));
    cambiarEstado(REINTENTO_PENDIENTE); tiempoProximoReintento=millis()+TIEMPO_ESPERA_REINTENTO;
  }
}

void forzarPosicionSegura(){
  LOG_ACTION("Forzando posición 45°");
  if(anguloActual>50){
    digitalWrite(RELAY_BAJAR,HIGH); uint32_t t=millis();
    while(anguloActual>45 && millis()-t<30000){anguloActual=leerAnguloActual(); delay(500);}
    digitalWrite(RELAY_BAJAR,LOW);
  }else if(anguloActual<40){
    digitalWrite(RELAY_SUBIR,HIGH); uint32_t t=millis();
    while(anguloActual<45 && millis()-t<30000){anguloActual=leerAnguloActual(); delay(500);}
    digitalWrite(RELAY_SUBIR,LOW);
  }
  LOG_ACTION("Posición "+String(anguloActual,1)+"°");
}

void reiniciarSistema(){
  LOG_ACTION("Reiniciando subsistemas"); inicializarEthernet(); sincronizarHoraNTP(); inicializarModbus();
  contadorReintentos=0; tiempoProximoReintento=0; anguloActual=leerAnguloActual(); cambiarEstado(OPERATIVO);
  LOG_INFO("Reinicio completo");
}

void cambiarEstado(SistemaEstado nuevo)
{
    if (estado == nuevo) return;     // nada que hacer

    LOG_ACTION("Estado: " + obtenerNombreEstado(estado) +
               " → "    + obtenerNombreEstado(nuevo));

    /* apaga todo antes de decidir lo que toca */
    digitalWrite(LED_SUBIR, LOW);
    digitalWrite(LED_BAJAR, LOW);
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(RELAY_SUBIR, LOW);
    digitalWrite(RELAY_BAJAR, LOW);

    /* --------- decisiones según el nuevo estado --------- */
    if (nuevo == MOVIENDO_ARRIBA)
    {
        /* ---------- filtro END-STOP ---------- */
        if (endstopSuperiorActivo()) {                        //  ← NUEVO
            LOG_ERROR("Intento de SUBIR con end-stop activo");
            /* Te quedas en el estado anterior; 
               no enciendes relé ni LED y sales */
            return;   //  ───────►  fin de la función
        }

        /* activación normal de SUBIR */
        digitalWrite(LED_SUBIR, HIGH);
        digitalWrite(RELAY_SUBIR, HIGH);
    }

    else if (nuevo == MOVIENDO_ABAJO)
    {
        digitalWrite(LED_BAJAR, HIGH);
        digitalWrite(RELAY_BAJAR, HIGH);
    }

    else if (nuevo == ENDSTOP_VERTICAL ||
             nuevo == ERROR_NTP        ||
             nuevo == ERROR_MODBUS     ||
             nuevo == ERROR_SEGURIDAD  ||
             nuevo == ERROR_MOVIMIENTO ||
             nuevo == REINTENTO_PENDIENTE)
    {
        digitalWrite(LED_ERROR, HIGH);
    }

    /* si realmente iniciamos un movimiento, guarda ángulo de partida */
    if (nuevo == MOVIENDO_ARRIBA || nuevo == MOVIENDO_ABAJO)
        anguloInicioMovimiento = anguloActual;

    /* actualiza variables de estado */
    estadoAnterior = estado;
    estado         = nuevo;
    ultimoMovimientoTiempo = millis();
}

String obtenerNombreEstado(SistemaEstado e){
  switch(e){
    case INICIALIZANDO: return "INICIALIZANDO";
    case OPERATIVO: return "OPERATIVO";
    case MOVIENDO_ARRIBA: return "MOV_ARRIBA";
    case MOVIENDO_ABAJO:  return "MOV_ABAJO";
    case ALINEADO: return "ALINEADO";
    case ERROR_NTP: return "ERR_NTP";
    case ERROR_MODBUS: return "ERR_MODBUS";
    case ERROR_SEGURIDAD: return "ERR_SEGURIDAD";
    case ERROR_MOVIMIENTO: return "ERR_MOV";
    case REINTENTO_PENDIENTE: return "REINTENTO";
    case MODO_ESPERA: return "ESPERA";
    default: return "DESCONOC";
  }
}

/* ---------- registrarEvento ---------- */
void registrarEvento(const String &msg, bool crit) {
    String ts = timeClient.getFormattedTime();
    if(ts.length() == 0) ts = "00:00:00";
    Serial.print("[" + ts + "] "); // ← Ya muestra hora con offset
    Serial.print(crit ? "⛔ " : "ℹ️ ");
    Serial.println(msg);
    if(crit) {
        digitalWrite(LED_ERROR, HIGH);
        delay(100);
        digitalWrite(LED_ERROR, LOW);
    }
}