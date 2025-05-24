/*
 * [NASA SpaceTrek Weather Balloon Launch - Batteries w/ Thermistors Testing]
 * Starter program for using an Arduino Uno or MEGA 2560 with
 * an XBEE shield attached. Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then 
 * replace the shield to test with the Flight Computer (FC).
 * 
 * Date: Mar. 20th, 2025
 */

// Included Libraries
#include <Arduino.h>
// Include any additional libraries needed here

// Settings
#define PODID         1       // Must be 1, 2 or 3. This is the PODID that this experiment will respond to when the FC asks for pod data
#define maxMessage    96      // Maximum number of bytes that can be in the message sent to the FC

// Custom settings
#define TIMER_TIME    1000    // The number of milliseconds to wait before running the timer if statement again

// ================= Special Bytes =================
// These are used for the communication protocol with the FC
#define specialByte        250
#define ACKMarker          251
#define slaveStartMarker   252
#define slaveEndMarker     253
#define masterStartMarker  254
#define masterEndMarker    255
// =================================================

// ========== FC Communication Variables ==========
byte bytesRecvd = 0;
byte dataSentNum = 0;
byte dataRecvCount = 0;
byte numDataBytes = 0;

byte dataRecvd[maxMessage];
byte dataSend[maxMessage];
byte encodeBuffer[maxMessage];

byte dataSendCount = 0;
byte dataTotalSend = 0;

boolean returnData = false;
boolean reciveCommand = false;

char xbeeInBuffer = "";
byte inCharBuffer = 0;

// POD data buffer
byte data[40];

// Sensor value holders using union
union u_float {
  float value;
  byte bytes[4];
} val_01, val_02, val_03, val_04, val_05, val_06, val_07, val_08, val_09, val_10;

// ========== User Variables ==========
uint32_t timer = 0;
const int tempPins[4] = {A8, A9, A10, A11};
const int voltPins[4] = {A12, A13, A14, A15};

const float BETA = 3950;
const float R25 = 9983.1;
const float T0 = 298.15;
const float SERIES_RESISTOR = 10000;
const int MISSING_SENSOR_THRESHOLD = 10;
const int MISSING_BATTERY_THRESHOLD = 5;

void setup() {
  Serial.begin(57600);
  while (!Serial);

  val_01.value = 0.0;
  val_02.value = 0.0;
  val_03.value = 0.0;
  val_04.value = 0.0;
  val_05.value = 0.0;
  val_06.value = 0.0;
  val_07.value = 0.0;
  val_08.value = 0.0;
  val_09.value = 0.0;
  val_10.value = 0.0;

  timer = millis();
}

void loop() {
  if ((millis() - timer) >= TIMER_TIME) {
    timer = millis();

    // ===== Voltage Reading =====
    for (int i = 0; i < 4; i++) {
      int rawVoltage = analogRead(voltPins[i]);
      float voltage = -1.0;
      if (rawVoltage >= MISSING_BATTERY_THRESHOLD) {
        voltage = (rawVoltage * 5.0) / 1023.0;
      }
      if (i == 0) val_01.value = voltage;
      else if (i == 1) val_02.value = voltage;
      else if (i == 2) val_03.value = voltage;
      else if (i == 3) val_04.value = voltage;
    }

    // ===== Temperature Reading =====
    for (int i = 0; i < 4; i++) {
      int rawADC = analogRead(tempPins[i]);
      float tempC = -273.15;
      if (rawADC >= MISSING_SENSOR_THRESHOLD) {
        float v = (rawADC * 5.0) / 1023.0;
        float R = SERIES_RESISTOR * (5.0 / v - 1);
        float tempK = (BETA * T0) / (BETA + (T0 * log(R / R25)));
        tempC = tempK - 273.15;
      }
      if (i == 0) val_05.value = tempC;
      else if (i == 1) val_06.value = tempC;
      else if (i == 2) val_07.value = tempC;
      else if (i == 3) val_08.value = tempC;
    }
  }

  // ======= XBEE FC Communication =======
  while (Serial.available()) {
    inCharBuffer = Serial.read();
    if (inCharBuffer == masterStartMarker) {
      reciveCommand = true;
    } else if (inCharBuffer == masterEndMarker) {
      reciveCommand = false;
      returnData = true;
      delay(10);
    } else {
      if (reciveCommand) {
        xbeeInBuffer = inCharBuffer;
      }
    }
  }

  if (returnData) {
    if (xbeeInBuffer == PODID) {
      numDataBytes = storeData();
      encodeData(numDataBytes);
      sendData(numDataBytes);
    }
    xbeeInBuffer = "";
    returnData = false;
  }
}

// ==================== Binary Data Functions ====================
// DO NOT MODIFY THESE

int storeData() {
  data[0] = val_01.bytes[0]; data[1] = val_01.bytes[1]; data[2] = val_01.bytes[2]; data[3] = val_01.bytes[3];
  data[4] = val_02.bytes[0]; data[5] = val_02.bytes[1]; data[6] = val_02.bytes[2]; data[7] = val_02.bytes[3];
  data[8] = val_03.bytes[0]; data[9] = val_03.bytes[1]; data[10] = val_03.bytes[2]; data[11] = val_03.bytes[3];
  data[12] = val_04.bytes[0]; data[13] = val_04.bytes[1]; data[14] = val_04.bytes[2]; data[15] = val_04.bytes[3];
  data[16] = val_05.bytes[0]; data[17] = val_05.bytes[1]; data[18] = val_05.bytes[2]; data[19] = val_05.bytes[3];
  data[20] = val_06.bytes[0]; data[21] = val_06.bytes[1]; data[22] = val_06.bytes[2]; data[23] = val_06.bytes[3];
  data[24] = val_07.bytes[0]; data[25] = val_07.bytes[1]; data[26] = val_07.bytes[2]; data[27] = val_07.bytes[3];
  data[28] = val_08.bytes[0]; data[29] = val_08.bytes[1]; data[30] = val_08.bytes[2]; data[31] = val_08.bytes[3];
  data[32] = val_09.bytes[0]; data[33] = val_09.bytes[1]; data[34] = val_09.bytes[2]; data[35] = val_09.bytes[3];
  data[36] = val_10.bytes[0]; data[37] = val_10.bytes[1]; data[38] = val_10.bytes[2]; data[39] = val_10.bytes[3];
  return 40;
}

void encodeData(byte numBytes) {
  dataTotalSend = 0;
  for (byte n = 0; n < numBytes; n++) {
    if (data[n] >= specialByte) {
      encodeBuffer[dataTotalSend++] = specialByte;
      encodeBuffer[dataTotalSend++] = data[n] - specialByte;
    } else {
      encodeBuffer[dataTotalSend++] = data[n];
    }
  }
}

void sendData(byte sendCount) {
  if (sendCount > 249) {
    Serial.println("ERROR: sendCount greater than 249");
    return;
  }
  Serial.write(slaveStartMarker);
  Serial.write(sendCount);
  Serial.write(encodeBuffer, dataTotalSend);
  Serial.write(slaveEndMarker);
}
