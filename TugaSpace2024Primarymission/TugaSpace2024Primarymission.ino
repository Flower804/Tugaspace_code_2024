//Lora
#include <LoRa.h>
#include <SPI.h>
#define ss 10
#define rst 7
#define dio0 6

//serial to RP0
static const int RXPin2 = D4, TXPin2 = D5;


//GPS
#include <TinyGPSPlus.h>
static const int RXPin = D3, TXPin = D2;
static const uint32_t GPSBaud = 38400;
// The TinyGPSPlus object
TinyGPSPlus gps;
#include <Adafruit_Sensor.h>


//I2C
#include <Wire.h>

//BME680
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;  // I2C


//MPU9250
#include "MPU9250.h"
//

MPU9250 mpu;


//ADXL375
#include <Adafruit_ADXL375.h>
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);


//VEML7700
#include "DFRobot_VEML7700.h"
DFRobot_VEML7700 als;


//Buzzer

#include <Buzzer.h>
Buzzer buzzer(25);


long previousMillis;
//estados do Cansat
enum state { PRE_LAUNCH,
             GPS_READY,
             LAUNCH_READY,
             LAUNCH,
             ROCKET_RELEASE,
             APOGGE,
             PARCHUTE_OPEN,
             SONDA_RELEASE,
             TOUCH_DOWN };


typedef struct Estrutura_dados {
  int millisegundos;
  double bmeTemp;
  double mpuTemp;
  double bmePressure;
  double bmeHumidity;
  float pressaoInicial;
  float bmeAltitude;
  float acel_x;
  float acel_y;
  float acel_z;
  float mpu_acel_x;
  float mpu_acel_y;
  float mpu_acel_z;
  float mpu_gyro_x;
  float mpu_gyro_y;
  float mpu_gyro_z;
  float mpu_yaw;
  float mpu_Pitch;
  float mpu_Roll;
  float mag_x;
  float mag_y;
  float mag_z;
  int estado;
  float aceleracaoTotal;
  float luz;
  float luzLancamento;
  double gps_Lat;
  double gps_Long;
  double gps_Alt;
  double gps_Velocidade;
  double gps_Direcao;
  double pressaoPontoMaisAlto;
  //flags

  bool isForaFoguete = false;
  bool isParaquedasAberto = false;
  bool isGPSSet = false;
  bool isPontoMaisAlto = false;
  bool isPontoMaisAltoGPS = false;

  bool isPressaoInicial = false;
  bool isTOUCH_DOWN = false;

} Estrutura_dados;
Estrutura_dados dados;


int freq = 433150000;
int counter = 0;

void setup() {
  dados.estado = 0;
  Serial.begin(115200);

  //Inicia GPS
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  //inicializa comunicacao com RP0
  Serial2.begin(9600, SERIAL_8N1, RXPin2, TXPin2);

  //setup LoRa transceiver module
  //Serial.println("LoRa Sender");

  LoRa.setPins(ss, rst, dio0);
  LoRa.setTxPower(20);  //setup LoRa transceiver module
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(31.25E3);


  while (!LoRa.begin(freq))  //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");


  //Inicia ADXL375

  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while (1)
      ;
  }
  //inicia mpu 9250
  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  //inicia BME

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  /* Initialise the ADXL */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while (1)
      ;
  }
  Beep();
}








void loop() {
  int interval = 900;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    //estadoCheck();
    dados.millisegundos = millis();





    lerADXL();
    lerLuz();
    avaliaEstado();
    lerBME();
    lerGPS();
    lerMPU();
    calculaAcceleracaoTotal();
    enviaLora();
    enviaSerial();
    enviarSerial2();
    if (dados.estado == 4) {
      Beep();
    }
  }
}


void Beep() {
  buzzer.begin(0);
  buzzer.sound(NOTE_A3, 200);
  buzzer.end(100);
}



void calculaAcceleracaoTotal() {
  dados.aceleracaoTotal = sqrt(pow(dados.acel_x, 2) + pow(dados.acel_y, 2) + pow(dados.acel_z, 2));
}

void avaliaEstado() {
  //situação de lançamento
  if (dados.aceleracaoTotal > 30) {
    dados.luzLancamento = dados.luz;
    dados.estado = 1;
  }
  //situação sair do fuguetão
  if ((dados.estado == 1) &&(dados.luz > 3 * dados.luzLancamento)) {
      dados.estado = 2;
    }
  //situação velocidade terminal
  if ((dados.estado == 2) &&(dados.aceleracaoTotal < 20)) {
      dados.estado = 3;
    }
  //situação atterrar
  if ( (dados.estado == 3) &&(dados.bmeAltitude < 100)) {
      dados.estado = 4;
    }
}
//le dados
void lerMPU() {
  mpu.update();
  dados.mpu_acel_x = mpu.getAccX();
  dados.mpu_acel_y = mpu.getAccY();
  dados.mpu_acel_z = mpu.getAccZ();
  dados.mpu_gyro_x = mpu.getGyroX();
  dados.mpu_gyro_y = mpu.getGyroY();
  dados.mpu_gyro_z = mpu.getGyroZ();
  dados.mpu_Pitch = mpu.getPitch();
  dados.mpu_Roll = mpu.getRoll();
  dados.mpu_yaw = mpu.getYaw();
}

void lerBME() {
  bme.performReading();
  dados.bmeTemp = bme.temperature;
  dados.bmePressure = bme.pressure;
  dados.bmeHumidity = bme.humidity;
  if (dados.isPressaoInicial == false) {
    dados.pressaoInicial = dados.bmePressure;
    dados.isPressaoInicial = true;
  };
  dados.bmeAltitude = calculaAltitude(dados.bmePressure);
}

void lerLuz() {
  float lux;
  als.getALSLux(lux);
  dados.luz = lux;
}


void lerADXL() {
  sensors_event_t event;
  accel.getEvent(&event);

  dados.acel_x = event.acceleration.x;
  dados.acel_y = event.acceleration.y;
  dados.acel_z = event.acceleration.z;


  /* Display the results (acceleration is measured in m/s^2) */
}








void enviaLora() {

  LoRa.beginPacket();  //Send LoRa packet to receiver
  LoRa.print("/ ");
  LoRa.print("* ");
  LoRa.print(dados.millisegundos);
  LoRa.print(", ");
  LoRa.print(dados.acel_x);
  LoRa.print(", ");
  LoRa.print(dados.acel_y);
  LoRa.print(", ");
  LoRa.print(dados.acel_z);
  LoRa.print(", ");
  LoRa.print(dados.aceleracaoTotal);
  LoRa.print(", ");
  LoRa.print(dados.bmeTemp);
  LoRa.print(", ");
  LoRa.print(dados.bmePressure);
  LoRa.print(", ");
  LoRa.print(dados.bmeHumidity);
  LoRa.print(", ");
  LoRa.print(dados.bmeAltitude);
  LoRa.print(", ");
  LoRa.print(dados.luz);
  LoRa.print(", ");
  LoRa.print(dados.estado);
  LoRa.print(", ");
  LoRa.print(dados.gps_Lat,6);
  LoRa.print(", ");
  LoRa.print(dados.gps_Long,6);
  LoRa.print(", ");
  LoRa.print(dados.gps_Alt);
  LoRa.print(", ");
  LoRa.print(dados.gps_Velocidade);
  LoRa.print(", ");
  LoRa.print(dados.gps_Direcao);
  LoRa.print("*");
  LoRa.println("/ ");
  LoRa.endPacket();
}
void enviaSerial() {




  Serial.print(dados.millisegundos);
  Serial.print(", ");
  Serial.print(dados.acel_x);
  Serial.print(", ");



  Serial.print(dados.acel_y);
  Serial.print(", ");

  Serial.print(dados.acel_z);
  Serial.print(", ");
  Serial.print(dados.aceleracaoTotal);
  Serial.print(", ");
  Serial.print(dados.bmeTemp);
  Serial.print(", ");
  Serial.print(dados.bmePressure);
  Serial.print(", ");
  Serial.print(dados.bmeHumidity);
  Serial.print(", ");
  Serial.print(dados.bmeAltitude);
  Serial.print(", ");
  Serial.print(dados.luz);

  Serial.print(", ");
  Serial.print(dados.estado);
  Serial.print(", ");

  Serial.print(dados.gps_Lat, 6);
  Serial.print(", ");
  Serial.print(dados.gps_Long, 6);
  Serial.print(", ");
  Serial.print(dados.gps_Alt);
  Serial.print(", ");
  Serial.print(dados.gps_Velocidade);
  Serial.print(", ");
  Serial.print(dados.gps_Direcao);
  Serial.println(", ");
}

void enviaSerial2() {
  Serial2.print(dados.millisegundos);
  Serial2.print(", ");
  Serial2.print(dados.acel_x);
  Serial2.print(", ");
  Serial2.print(dados.acel_y);
  Serial2.print(", ");
  Serial2.print(dados.acel_z);
  Serial2.print(", ");
  Serial2.print(dados.aceleracaoTotal);
  Serial2.print(", ");
  Serial2.print(dados.bmeTemp);
  Serial2.print(", ");
  Serial2.print(dados.bmePressure);
  Serial2.print(", ");
  Serial2.print(dados.bmeHumidity);
  Serial2.print(", ");
  Serial2.print(dados.bmeAltitude);
  Serial2.print(", ");
  Serial2.print(dados.luz);
  Serial2.print(", ");
  Serial2.print(dados.estado);
  Serial2.print(", ");
  Serial2.print(dados.gps_Lat, 6);
  Serial2.print(", ");
  Serial2.print(dados.gps_Long, 6);
  Serial2.print(", ");
  Serial2.print(dados.gps_Alt);
  Serial2.print(", ");
  Serial2.print(dados.gps_Velocidade);
  Serial2.print(", ");
  Serial2.println(dados.gps_Direcao);
}



void lerGPS() {
  smartDelay(100);
  dados.gps_Lat = gps.location.lat();
  dados.gps_Long = gps.location.lng();

  dados.gps_Alt = gps.altitude.meters();
  dados.gps_Direcao = gps.course.deg();


  dados.gps_Velocidade = gps.speed.mps();
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}


float calculaAltitude(float pressao) {
  float altitude = 44330 * (1 - pow(pressao / dados.pressaoInicial, 1 / 5.255));


  return altitude;
}
