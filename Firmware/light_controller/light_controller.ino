// ============================================================
//  Light Controller – ESP32-S3
//  Basado en diagrama de estados: s0, sM, sA, sDir, sF, sC, sS, sE
//  IDE: Arduino IDE  |  Board: ESP32-S3
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>   // Librería: Adafruit MPU6050 (Library Manager)
#include <Adafruit_Sensor.h>    // Dependencia de Adafruit Unified Sensor

// ─────────────────────────────────────────────
//  PINES
// ─────────────────────────────────────────────
#define PIN_LUZ_FRONTAL          1
#define PIN_LUZ_IZQ_FRONTAL     17
#define PIN_LUZ_DER_FRONTAL      2
#define PIN_LUZ_TRASERA_BAJA    21
#define PIN_LUZ_TRASERA_ALTA    14
#define PIN_LUZ_IZQ_TRASERA     13
#define PIN_LUZ_DER_TRASERA     47
#define PIN_I2C_SDA              5
#define PIN_I2C_SCL              4
#define PIN_TEMT6000             6   // Analógico
#define PIN_SWITCH_MODE         38
#define PIN_CLAXON_SWITCH        9
#define PIN_SWITCH_IZQ          46
#define PIN_SWITCH_DER           3
#define PIN_FRONT_LIGHT_SWITCH   8
#define PIN_CLAXON               7

// ─────────────────────────────────────────────
//  CANALES PWM (LEDC – ESP32)
// ─────────────────────────────────────────────
#define PWM_FREQ          1000   // Hz
#define PWM_RES              8   // bits → 0-255
// La nueva API ledcAttach() asigna el canal automáticamente por pin

// ─────────────────────────────────────────────
//  CONSTANTES DE COMPORTAMIENTO
// ─────────────────────────────────────────────
#define DELAY_LIGHTS        200   // ms – parpadeo direccionales

// Umbrales eje Y (m/s²) para modo automático
// Adafruit MPU6050 entrega aceleración en m/s² (±8g → ±78.4 m/s²)
#define UMBRAL_IZQ_Y      -3.0f   // girado a la izquierda si Y < este valor
#define UMBRAL_DER_Y       3.0f   // girado a la derecha  si Y > este valor

// Umbrales eje Z (m/s²) para detección de frenada
// Frenada: componente Z negativa al desacelerar
#define FRENO_LEVE       -2.0f   // frenada leve     → PWM 50%
#define FRENO_MODERADO   -5.0f   // frenada moderada → PWM 75%
// más negativo que FRENO_MODERADO → PWM 100%

// Intervalo de muestreo del MPU6050 (polling por millis)
#define MPU_SAMPLE_MS       20   // ms

// ─────────────────────────────────────────────
//  ESTADOS
// ─────────────────────────────────────────────
typedef enum {
  STATE_S0,    // Inicial / configuración
  STATE_SM,    // Manual
  STATE_SA,    // Automático
  STATE_SDIR,  // Control de direccionales
  STATE_SF,    // Control de luz frontal
  STATE_SC,    // Control de claxon
  STATE_SS,    // Control de stop
  STATE_SE     // Error
} State;

// ─────────────────────────────────────────────
//  VARIABLES GLOBALES
// ─────────────────────────────────────────────
volatile bool FLAG_SWITCH_MODE       = false;
volatile bool FLAG_CLAXON_SWITCH     = false;
volatile bool FLAG_SWITCH_IZQ        = false;
volatile bool FLAG_SWITCH_DER        = false;
volatile bool FLAG_FRONT_LIGHT_SWITCH= false;
volatile bool STOP_FLAG              = false;

// Adafruit MPU6050 entrega valores en m/s²  (accel) y rad/s (gyro)
float accelX = 0, accelY = 0, accelZ = 0;

uint8_t  front_pwm   = 128;   // 50% por defecto (0-255)
uint8_t  stop_pwm    = 0;

State    currentState = STATE_S0;
String   errorMsg     = "";

Adafruit_MPU6050 mpu;

// Temporización del muestreo del MPU6050 (polling con millis)
unsigned long lastMpuSample  = 0;

// Para parpadeo sin delay (sDir, sS)
unsigned long lastToggleDir  = 0;
unsigned long lastToggleSS   = 0;
bool          dirLightsOn    = false;
bool          ssLightsSwap   = false;

// ─────────────────────────────────────────────
//  ISR – Interrupciones de señales digitales
// ─────────────────────────────────────────────
void IRAM_ATTR isrSwitchMode()      { FLAG_SWITCH_MODE        = digitalRead(PIN_SWITCH_MODE); }
void IRAM_ATTR isrClaxon()          { FLAG_CLAXON_SWITCH      = digitalRead(PIN_CLAXON_SWITCH); }
void IRAM_ATTR isrSwitchIzq()       { FLAG_SWITCH_IZQ         = digitalRead(PIN_SWITCH_IZQ); }
void IRAM_ATTR isrSwitchDer()       { FLAG_SWITCH_DER         = digitalRead(PIN_SWITCH_DER); }
void IRAM_ATTR isrFrontLightSwitch(){ FLAG_FRONT_LIGHT_SWITCH = digitalRead(PIN_FRONT_LIGHT_SWITCH); }

// ─────────────────────────────────────────────
//  Función de muestreo del MPU6050 (polling)
//  Llamar desde el loop principal cada MPU_SAMPLE_MS
// ─────────────────────────────────────────────
void mpuSample() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;

  // Detectar frenada: componente Z negativa (por debajo del umbral leve)
  // Umbrales en m/s²: FRENO_LEVE=-3, FRENO_MODERADO=-6
  if (accelZ < FRENO_LEVE) {
    STOP_FLAG = true;
    if      (accelZ > FRENO_LEVE)     stop_pwm = 128;   // leve     50%
    else if (accelZ > FRENO_MODERADO) stop_pwm = 191;   // moderada 75%
    else                               stop_pwm = 255;   // fuerte   100%
  } else {
    STOP_FLAG = false;
    stop_pwm  = 0;
  }
}

// ─────────────────────────────────────────────
//  HELPERS – PWM y GPIO
// ─────────────────────────────────────────────
void setFrontPWM(uint8_t duty) {
  ledcWrite(PIN_LUZ_FRONTAL, duty);
}

void setStopHighPWM(uint8_t duty) {
  ledcWrite(PIN_LUZ_TRASERA_ALTA, duty);
}

void apagaTodasLuces() {
  setFrontPWM(0);
  setStopHighPWM(0);
  digitalWrite(PIN_LUZ_IZQ_FRONTAL,  LOW);
  digitalWrite(PIN_LUZ_DER_FRONTAL,  LOW);
  digitalWrite(PIN_LUZ_TRASERA_BAJA, LOW);
  digitalWrite(PIN_LUZ_IZQ_TRASERA,  LOW);
  digitalWrite(PIN_LUZ_DER_TRASERA,  LOW);
  digitalWrite(PIN_CLAXON,           LOW);
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  currentState = STATE_S0;
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  // Muestreo periódico del MPU6050 (polling no bloqueante)
  unsigned long now = millis();
  if (now - lastMpuSample >= MPU_SAMPLE_MS) {
    lastMpuSample = now;
    mpuSample();
  }

  switch (currentState) {
    case STATE_S0:  state_s0();   break;
    case STATE_SM:  state_sM();   break;
    case STATE_SA:  state_sA();   break;
    case STATE_SDIR: state_sDir(); break;
    case STATE_SF:  state_sF();   break;
    case STATE_SC:  state_sC();   break;
    case STATE_SS:  state_sS();   break;
    case STATE_SE:  state_sE();   break;
  }
}

// ============================================================
//  IMPLEMENTACIÓN DE ESTADOS
// ============================================================

// ─────────────────────────────────────────────
//  s0 – Inicial y configuración
// ─────────────────────────────────────────────
void state_s0() {
  Serial.println("[s0] Iniciando sistema...");

  // 1. Configurar pines de salida
  pinMode(PIN_LUZ_FRONTAL,        OUTPUT);
  pinMode(PIN_LUZ_IZQ_FRONTAL,    OUTPUT);
  pinMode(PIN_LUZ_DER_FRONTAL,    OUTPUT);
  pinMode(PIN_LUZ_TRASERA_BAJA,   OUTPUT);
  pinMode(PIN_LUZ_TRASERA_ALTA,   OUTPUT);
  pinMode(PIN_LUZ_IZQ_TRASERA,    OUTPUT);
  pinMode(PIN_LUZ_DER_TRASERA,    OUTPUT);
  pinMode(PIN_CLAXON,             OUTPUT);

  // 2. Configurar pines de entrada
  pinMode(PIN_SWITCH_MODE,         INPUT);
  pinMode(PIN_CLAXON_SWITCH,       INPUT);
  pinMode(PIN_SWITCH_IZQ,          INPUT);
  pinMode(PIN_SWITCH_DER,          INPUT);
  pinMode(PIN_FRONT_LIGHT_SWITCH,  INPUT);

  // 3. Configurar canales PWM (nueva API LEDC: ledcAttach unifica setup y attach)
  if (!ledcAttach(PIN_LUZ_FRONTAL,      PWM_FREQ, PWM_RES)) {
    errorMsg = "LEDC: fallo al configurar luz frontal";
    currentState = STATE_SE;
    return;
  }
  if (!ledcAttach(PIN_LUZ_TRASERA_ALTA, PWM_FREQ, PWM_RES)) {
    errorMsg = "LEDC: fallo al configurar luz trasera alta";
    currentState = STATE_SE;
    return;
  }

  // 4. I2C y MPU6050 (Adafruit) – solo SDA/SCL, sin pin de interrupción
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!mpu.begin()) {
    errorMsg = "MPU6050 no responde";
    currentState = STATE_SE;
    return;
  }
  // Rangos de trabajo: ±8g accel, 500°/s gyro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[s0] MPU6050 OK");

  // 5. Verificar TEMT6000 (lectura analógica básica)
  int temt = analogRead(PIN_TEMT6000);
  Serial.print("[s0] TEMT6000 lectura: ");
  Serial.println(temt);
  // analogRead siempre devuelve >= 0; se puede ampliar con umbral si es necesario

  // 6. Adjuntar interrupciones para señales digitales
  attachInterrupt(digitalPinToInterrupt(PIN_SWITCH_MODE),        isrSwitchMode,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CLAXON_SWITCH),      isrClaxon,           CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SWITCH_IZQ),         isrSwitchIzq,        CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SWITCH_DER),         isrSwitchDer,        CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_FRONT_LIGHT_SWITCH), isrFrontLightSwitch, CHANGE);

  // 7. Inicializar temporizador de muestreo del MPU6050
  lastMpuSample = millis();

  Serial.println("[s0] Configuracion completada → sM");
  currentState = STATE_SM;
}

// ─────────────────────────────────────────────
//  sM – Estado Manual
// ─────────────────────────────────────────────
void state_sM() {
  // Luz frontal siempre al 50%
  front_pwm = 128;
  setFrontPWM(front_pwm);

  // Luz trasera baja siempre encendida
  digitalWrite(PIN_LUZ_TRASERA_BAJA, HIGH);

  // --- Evaluación de transiciones (prioridad descendente) ---

  // Modo automático
  if (FLAG_SWITCH_MODE) {
    apagaTodasLuces();
    currentState = STATE_SA;
    return;
  }

  // Frenada
  if (STOP_FLAG) {
    currentState = STATE_SS;
    return;
  }

  // Direccionales (no pueden estar las dos a la vez)
  if (FLAG_SWITCH_IZQ && FLAG_SWITCH_DER) {
    errorMsg = "sM: SWITCH_IZQ y SWITCH_DER activos simultaneamente";
    currentState = STATE_SE;
    return;
  }
  if (FLAG_SWITCH_IZQ || FLAG_SWITCH_DER) {
    lastToggleDir = millis();
    dirLightsOn   = false;
    currentState  = STATE_SDIR;
    return;
  }

  // Luz frontal al 100%
  if (FLAG_FRONT_LIGHT_SWITCH) {
    currentState = STATE_SF;
    return;
  }

  // Claxon
  if (FLAG_CLAXON_SWITCH) {
    currentState = STATE_SC;
    return;
  }

  // Sin cambios: permanecer en sM
}

// ─────────────────────────────────────────────
//  sA – Estado Automático
// ─────────────────────────────────────────────
void state_sA() {
  // Volver a Manual si se desactiva SWITCH_MODE
  if (!FLAG_SWITCH_MODE) {
    apagaTodasLuces();
    front_pwm = 128;
    setFrontPWM(front_pwm);
    digitalWrite(PIN_LUZ_TRASERA_BAJA, HIGH);
    currentState = STATE_SM;
    return;
  }

  // Detectar frenada (STOP_FLAG actualizado por polling en loop)
  if (STOP_FLAG) {
    currentState = STATE_SS;
    return;
  }

  // Control de direccionales por MPU6050 eje Y
  if (accelY < UMBRAL_IZQ_Y) {
    // Girado a la izquierda
    digitalWrite(PIN_LUZ_IZQ_FRONTAL, HIGH);
    digitalWrite(PIN_LUZ_IZQ_TRASERA, HIGH);
    digitalWrite(PIN_LUZ_DER_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_DER_TRASERA, LOW);
  } else if (accelY > UMBRAL_DER_Y) {
    // Girado a la derecha
    digitalWrite(PIN_LUZ_DER_FRONTAL, HIGH);
    digitalWrite(PIN_LUZ_DER_TRASERA, HIGH);
    digitalWrite(PIN_LUZ_IZQ_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_IZQ_TRASERA, LOW);
  } else {
    // Centrado
    digitalWrite(PIN_LUZ_IZQ_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_IZQ_TRASERA, LOW);
    digitalWrite(PIN_LUZ_DER_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_DER_TRASERA, LOW);
  }
}

// ─────────────────────────────────────────────
//  sDir – Control de Direccionales
// ─────────────────────────────────────────────
void state_sDir() {
  // Error: ambas activas
  if (FLAG_SWITCH_IZQ && FLAG_SWITCH_DER) {
    apagaTodasLuces();
    errorMsg = "sDir: Ambas direccionales activas";
    currentState = STATE_SE;
    return;
  }

  // Fin del estado: ninguna activa → volver a sM
  if (!FLAG_SWITCH_IZQ && !FLAG_SWITCH_DER) {
    digitalWrite(PIN_LUZ_IZQ_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_IZQ_TRASERA, LOW);
    digitalWrite(PIN_LUZ_DER_FRONTAL, LOW);
    digitalWrite(PIN_LUZ_DER_TRASERA, LOW);
    currentState = STATE_SM;
    return;
  }

  // Parpadeo no bloqueante con millis()
  unsigned long now = millis();
  if (now - lastToggleDir >= DELAY_LIGHTS) {
    lastToggleDir = now;
    dirLightsOn   = !dirLightsOn;

    if (FLAG_SWITCH_IZQ) {
      digitalWrite(PIN_LUZ_IZQ_FRONTAL, dirLightsOn ? HIGH : LOW);
      digitalWrite(PIN_LUZ_IZQ_TRASERA, dirLightsOn ? HIGH : LOW);
      // Apagar lado contrario por seguridad
      digitalWrite(PIN_LUZ_DER_FRONTAL, LOW);
      digitalWrite(PIN_LUZ_DER_TRASERA, LOW);
    } else if (FLAG_SWITCH_DER) {
      digitalWrite(PIN_LUZ_DER_FRONTAL, dirLightsOn ? HIGH : LOW);
      digitalWrite(PIN_LUZ_DER_TRASERA, dirLightsOn ? HIGH : LOW);
      digitalWrite(PIN_LUZ_IZQ_FRONTAL, LOW);
      digitalWrite(PIN_LUZ_IZQ_TRASERA, LOW);
    }
  }

  // Seguir monitoreando frenada incluso en sDir
  if (STOP_FLAG) {
    // Salir temporalmente a sS; al volver se retomará sDir
    currentState = STATE_SS;
  }
}

// ─────────────────────────────────────────────
//  sF – Control de Luz Frontal (100%)
// ─────────────────────────────────────────────
void state_sF() {
  front_pwm = 255;   // 100% duty cycle
  setFrontPWM(front_pwm);

  // Permanecer hasta que se suelte el switch
  if (!FLAG_FRONT_LIGHT_SWITCH) {
    front_pwm = 128;
    setFrontPWM(front_pwm);
    currentState = STATE_SM;
  }
  // Si ocurre error, se puede agregar aquí lógica adicional
}

// ─────────────────────────────────────────────
//  sC – Control de Claxon
// ─────────────────────────────────────────────
void state_sC() {
  digitalWrite(PIN_CLAXON, HIGH);

  if (!FLAG_CLAXON_SWITCH) {
    digitalWrite(PIN_CLAXON, LOW);
    currentState = STATE_SM;
  }
}

// ─────────────────────────────────────────────
//  sS – Control de Stop
// ─────────────────────────────────────────────
void state_sS() {
  // Determinar nivel de frenada y duty cycle
  uint8_t duty;
  if      (accelZ > FRENO_LEVE)     duty = 128;   // leve     50%
  else if (accelZ > FRENO_MODERADO) duty = 191;   // moderada 75%
  else                               duty = 255;   // fuerte   100%

  // Parpadeo no bloqueante de la luz trasera alta
  unsigned long now = millis();
  if (now - lastToggleSS >= DELAY_LIGHTS) {
    lastToggleSS  = now;
    ssLightsSwap  = !ssLightsSwap;

    if (ssLightsSwap) {
      setStopHighPWM(duty);
      digitalWrite(PIN_LUZ_TRASERA_BAJA, LOW);
    } else {
      setStopHighPWM(0);
      digitalWrite(PIN_LUZ_TRASERA_BAJA, HIGH);
    }
  }

  // Fin de frenada → volver a sM
  if (!STOP_FLAG) {
    setStopHighPWM(0);
    digitalWrite(PIN_LUZ_TRASERA_BAJA, HIGH);
    currentState = STATE_SM;
  }
}

// ─────────────────────────────────────────────
//  sE – Estado de Error (loop infinito)
// ─────────────────────────────────────────────
void state_sE() {
  // Apagar todo por seguridad
  apagaTodasLuces();

  // Reportar error indefinidamente por serial
  while (true) {
    Serial.print("[ERROR] ");
    Serial.println(errorMsg);
    delay(1000);
  }
}