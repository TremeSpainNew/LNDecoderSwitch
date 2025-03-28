#include <Arduino.h>
#include <LocoNet.h>
#include <EEPROM.h>
#include <config.h>

#if tipoDispositivo == 0
#include <PCF8574.h>
PCF8574 pcf8574;
#define numSwitch 4
#define nLNCV (numSwitch * 2 + 1)  // LNCV0 = dirección + 2 LNCV por cada desvío
int conexionTurnout[numSwitch * 2] = {2, 3, 4, 5, 6, 9, 10, 11};  // Pines donde se conectan los switch

void leerEstadosDesvios(bool inicial = false) {
  static uint8_t estadoAnterior = 0xFF;  // Variable estática para almacenar el último estado conocido

  if (lncv_R(3) == 1) { // Solo si la LNCV 3 está habilitada
    uint8_t estados = pcf8574.read();

    if (inicial || estados != estadoAnterior) {  // Enviar todos los estados solo al iniciar o si hubo cambios
      for (int i = 0; i < numSwitch; i++) {
        bool cerrado = !bitRead(estados, i * 2);
        bool abierto = !bitRead(estados, i * 2 + 1);
        bool cerradoPrev = !bitRead(estadoAnterior, i * 2);
        bool abiertoPrev = !bitRead(estadoAnterior, i * 2 + 1);

        if (inicial || cerrado != cerradoPrev || abierto != abiertoPrev) {  // Solo reportar cambios
          Serial.print("📢 Desvío ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(cerrado ? "Cerrado " : "");
          Serial.print(abierto ? "Abierto " : "");
          Serial.println();

          // Reportar estado SOLO si cambió
          reportSwitchOutputs(myAddress + i, cerrado, abierto);
        }
      }

      estadoAnterior = estados;  // Actualizar estado anterior solo después de procesar cambios
    }
  }
}

#elif tipoDispositivo == 1
#define numSwitch 8
#define nLNCV (numSwitch * 2 + 1)  // LNCV0 = dirección + 2 LNCV por cada desvío
int conexionTurnout[numSwitch * 2] = {2, 3, 4, 5, 6, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5};  // Pines donde se conectan los switch

#else
#error "Valor no válido para tipoDispositivo. Debe ser 0 o 1."
#endif

lnMsg *LnPacket;
LocoNetCVClass lnCV;

struct SwitchState {
  uint8_t pin;            // Pin de activación
  bool active;            // Si está activo
  uint32_t startTime;     // Tiempo de activación
  uint16_t duration;      // Duración de la activación
};

SwitchState switches[numSwitch * 2];  // Arreglo para los desvíos

volatile uint16_t myAddress;
volatile uint16_t delayTime;
volatile int tipoDesvio;

int numArt = 5030;
bool programmingMode;
uint8_t estadoAnterior = 0xFF;

/**
 * LNCV 0: Dirección del módulo
 * LNCV 1: Tipo de desvío
 * LNCV 2: Tiempo de activación de la bobina de los desvíos
 */

// Guardar LNCV en EEPROM
void lncv_W(uint16_t Adr, uint16_t val) {
  unsigned int Address = Adr * 2;
  EEPROM.update(Address, val & 0xFF);   // LSB
  EEPROM.update(Address + 1, val >> 8); // MSB
}

// Leer LNCV desde EEPROM
uint16_t lncv_R(uint16_t Adr) {
  unsigned int Address = Adr * 2;
  return EEPROM.read(Address) | (EEPROM.read(Address + 1) << 8);
}

// Reset LNCV
void resetLNCV() {
  for (int n = 1; n < nLNCV; n++) {
    lncv_W(n, 0);
  }
  lncv_W(0, 1);
}

void reportSwitchOutputs(uint8_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
  uint16_t tempAddress = Address - 1; // Restar 1 sin modificar la variable original
  uint8_t AddrH = ((tempAddress >> 8) & 0x0F) | (ClosedOutput ? 0x20 : 0) | (ThrownOutput ? 0x10 : 0);
  uint8_t AddrL = (tempAddress & 0x7F);  // Ajuste correcto para dirección de accesorio

  Serial.print("📡 Enviando estado de desvío -> Dirección: ");
  Serial.print(Address);
  Serial.print(" | Cerrado: ");
  Serial.print(ClosedOutput);
  Serial.print(" | Abierto: ");
  Serial.println(ThrownOutput);

  LocoNet.send(OPC_SW_REP, AddrL, AddrH);
}

// Configuración inicial
void setup() {
  Serial.begin(9600);
  
  if (lncv_R(0) == 65534) resetLNCV();

  LocoNet.init(7);
  #if tipoDispositivo == 0
    pcf8574.begin(0x38);
    leerEstadosDesvios(true);
  #endif
  
  myAddress = lncv_R(0);
  delayTime = lncv_R(2);
  tipoDesvio = lncv_R(1);

  Serial.print("📌 Dirección: "); Serial.println(myAddress);

  programmingMode = false;

  for (int i = 0; i < numSwitch * 2; i++) {
    pinMode(conexionTurnout[i], OUTPUT);
    switches[i] = { static_cast<uint8_t>(conexionTurnout[i]), false, 0, 0 };
  }
}

void loop() {
  LnPacket = LocoNet.receive();
  if (LnPacket) {
    uint8_t packetConsumed = LocoNet.processSwitchSensorMessage(LnPacket);
    if (packetConsumed == 0) {
      packetConsumed = lnCV.processLNCVMessage(LnPacket);
    }
  }

  uint32_t now = millis();
  for (int i = 0; i < numSwitch * 2; i++) {
    if (switches[i].active && now - switches[i].startTime >= switches[i].duration) {
      digitalWrite(switches[i].pin, LOW);
      switches[i].active = false;
      Serial.print("⏹ Desvío ");
      Serial.print(i);
      Serial.println(" desactivado");
    }
  }
  #if tipoDispositivo == 0
    leerEstadosDesvios(false);
  #endif
}

// Manejo de solicitudes de cambio de desvío
void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction) {
  Serial.print("📌 Recibida solicitud para dirección ");
  Serial.print(Address);
  Serial.println(Direction ? " - Cerrado" : " - Abierto");
  Serial.println(Output);

  if (Address >= myAddress && Address < myAddress + numSwitch) {
    Serial.println("✅ Dirección válida.");

    if (Output == 16) {
      uint16_t index = Address - myAddress;
      uint16_t tipoDesvio = lncv_R(index * 2 + 1);
      uint16_t delayTime = lncv_R(index * 2 + 2);

      Serial.print("🔧 Tipo: ");
      Serial.print(tipoDesvio);
      Serial.print(" - ⏳ Tiempo: ");
      Serial.println(delayTime);

      int indexSwitch = index * 2 + (Direction ? 0 : 1);
      
      if (tipoDesvio == 1) {  // Bobina
        if (!switches[indexSwitch].active) {
          switches[indexSwitch].active = true;
          switches[indexSwitch].startTime = millis();
          switches[indexSwitch].duration = delayTime;
          digitalWrite(conexionTurnout[indexSwitch], HIGH);

          Serial.print("✅ Activando bobina en pin ");
          Serial.println(conexionTurnout[indexSwitch]);
        }
      } else if (tipoDesvio == 2) { // Motor
        digitalWrite(conexionTurnout[index * 2], Direction);
        digitalWrite(conexionTurnout[index * 2 + 1], !Direction);
        Serial.println("🔄 Motor activado.");
      }
    }
  } else {
    Serial.println("❌ Dirección fuera de rango.");
  }
}

// Inicio de programación LNCV
int8_t notifyLNCVprogrammingStart(uint16_t & ArtNr, uint16_t & ModuleAddress) {
  if (ArtNr == numArt) {
    if (ModuleAddress == lncv_R(0)) {
      programmingMode = true;
      return LNCV_LACK_OK;
    } else if (ModuleAddress == 0xFFFF) {
      ModuleAddress = lncv_R(0);
      return LNCV_LACK_OK;
    }
    return -1;
  }
}

// Lectura de valores LNCV
int8_t notifyLNCVread(uint16_t ArtNr, uint16_t lncvAddress, uint16_t & lncvValue) {
  if (programmingMode && ArtNr == numArt && lncvAddress < nLNCV) {
    lncvValue = lncv_R(lncvAddress);
    Serial.print("📥 LNCV Read: ");
    Serial.println(lncvValue);
    return LNCV_LACK_OK;
  }
  return -1;
}

// Escritura de valores LNCV
int8_t notifyLNCVwrite(uint16_t ArtNr, uint16_t lncvAddress, uint16_t lncvValue) {
  if (programmingMode && ArtNr == numArt && lncvAddress < nLNCV) {
    lncv_W(lncvAddress, lncvValue);
    Serial.print("📤 LNCV Write: ");
    Serial.println(lncvValue);
    return LNCV_LACK_OK;
  }
  return -1;
}

// Fin de programación LNCV
void notifyLNCVprogrammingStop(uint16_t ArtNr, uint16_t ModuleAddress) {
  if (programmingMode && ArtNr == numArt && ModuleAddress == lncv_R(0)) {
    programmingMode = false;
  }
}
