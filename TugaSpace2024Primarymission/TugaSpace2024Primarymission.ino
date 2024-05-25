#include <iostream>
#include <cstring>
#include <string>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//-----------COMMENTS--------------
// Welp this was something, and I mean ALL of it was something.
// I don't know how to feel avout having "finished" this code, and say finished in that way because it's my second time making this code since the original was
//   lost, if you don't know that story, I sugest you read the news paper attatched to this repo.
// So yea, I probably won't touch this code for a long longggg time, or ever again lol.
// All the memories will stay in my head, god knows how much time I spent making this and how much coffe it took lol (real ones from cansat will know what I mean)
// ass: Your dearest Flower :P
//      and Moita(like we're not the same person and Flower is just a nickname)

//TODO: think of how to implement sensor reading
//Done(2x): (01/05/2024) & (24/05/2024)

//Task handles
TaskHandle_t esp_turn_handle = NULL; //_2_
TaskHandle_t HUB_handle = NULL;      //_1_

//Hardware Stuff for Pc to tester ESP32 tester
int analogPin= 4;
int outputPin = 2;
int inputValue= 0;

//--------Handlers-------
boolean Busy = false;
boolean Turn = false;

//--------Serial Stuff---
String inputString = "";
String State;
boolean stringComplete = false;
boolean progresse;
int wait_time = 500;

//--------turns----------
String esp_Jessie;

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


//--------sensor-data----
struct dados_sense {
  double bmeTemp;
  double mpuTemp;
  float pressaoInicial;
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
  float luzLancamento;
  double pressaoPontoMaisAlto;
} dados_sense dados;

struct Estrutura_sender {
  int millisegundos;
  float acel_x;
  float acel_y;
  float acel_z;
  float aceleracaoTotal;
  double bmeTemp;
  double bmePressure;
  double bmeHumidity;
  float bmeAltitude;
  float luz;
  int estado;
  double gps_Lat;
  double gps_Long;
  double gps_Alt;
  double gps_Velocidade;
  double gps_Direcao;
} Estrutura_sender dados_send

typedef struct Estrutura_dados {
  //flags

  bool isForaFoguete = false;
  bool isParaquedasAberto = false;
  bool isGPSSet = false;
  bool isPontoMaisAlto = false;
  bool isPontoMaisAltoGPS = false;

  bool isPressaoInicial = false;
  bool isTOUCH_DOWN = false;

} Estrutura_dados;
Estrutura_dados dados_flag;


int freq = 433150000;
int counter = 0;

//voids for sensor reading
void Beep() { //Buzzer
  buzzer.begin(0);
  buzzer.sound(NOTE_A3, 200);
  buzzer.end(100);
}
void lerMPU() { //read positional data
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
void lerBME() { //read temperature, pressure and humidity
  bme.performReading();
  dados_sender.bmeTemp = bme.temperature;
  dados_sender.bmePressure = bme.pressure;
  dados_sender.bmeHumidity = bme.humidity;
  if (dados_flag.isPressaoInicial == false) {
    dados_flag.pressaoInicial = dados_sender.bmePressure;
    dados_flag.isPressaoInicial = true;
  };
  dados_sender.bmeAltitude = calculaAltitude(dados_sender.bmePressure);
}
void lerLuz() { //read luminosity
  float lux;
  als.getALSLux(lux);
  dados_sender.luz = lux;
}
void lerADXL() { //read accelarations
  sensors_event_t event;
  accel.getEvent(&event);

  dados_sender.acel_x = event.acceleration.x;
  dados_sender.acel_y = event.acceleration.y;
  dados_sender.acel_z = event.acceleration.z;


  /* Display the results (acceleration is measured in m/s^2) */
}
void calculaAcceleracaoTotal() {
  dados_sender.aceleracaoTotal = sqrt(pow(dados_sender.acel_x, 2) + pow(dados_sender.acel_y, 2) + pow(dados_sender.acel_z, 2));
}
void lerGPS() {
  smartDelay(100);
  dados_sender.gps_Lat = gps.location.lat();
  dados_sender.gps_Long = gps.location.lng();
  dados_sender.gps_Alt = gps.altitude.meters();
  dados_sender.gps_Direcao = gps.course.deg();


  dados_sender.gps_Velocidade = gps.speed.mps();
}
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
void avaliaEstado() {
  //situação de lançamento
  if (dados_sender.aceleracaoTotal > 30) {
    dados.luzLancamento = dados_sender.luz;
    dados_sender.estado = 1;
  }
  //situação sair do fuguetão
  if ((dados_sender.estado == 1) &&(dados_sender.luz > 3 * dados.luzLancamento)) {
      dados_sender.estado = 2;
    }
  //situação velocidade terminal
  if ((dados_sender.estado == 2) &&(dados_sender.aceleracaoTotal < 20)) {
      dados_sender.estado = 3;
    }
  //situação atterrar
  if ( (dados_sender.estado == 3) &&(dados_sender.bmeAltitude < 100)) {
      dados_sender.estado = 4;
    }
}

//--------Serial Print Voids-------- 
void enviaLora() {
  LoRa.beginPacket();  //Send LoRa packet to receiver
  LoRa.print("/ ");
  LoRa.print("* ");
  LoRa.print(dados_sender.millisegundos);
  LoRa.print(", ");
  LoRa.print(dados_sender.acel_x);
  LoRa.print(", ");
  LoRa.print(dados_sender.acel_y);
  LoRa.print(", ");
  LoRa.print(dados_sender.acel_z);
  LoRa.print(", ");
  LoRa.print(dados_sender.aceleracaoTotal);
  LoRa.print(", ");
  LoRa.print(dados_sender.bmeTemp);
  LoRa.print(", ");
  LoRa.print(dados_sender.bmePressure);
  LoRa.print(", ");
  LoRa.print(dados_sender.bmeHumidity);
  LoRa.print(", ");
  LoRa.print(dados_sender.bmeAltitude);
  LoRa.print(", ");
  LoRa.print(dados_sender.luz);
  LoRa.print(", ");
  LoRa.print(dados_sender.estado);
  LoRa.print(", ");
  LoRa.print(dados_sender.gps_Lat,6);
  LoRa.print(", ");
  LoRa.print(dados_sender.gps_Long,6);
  LoRa.print(", ");
  LoRa.print(dados_sender.gps_Alt);
  LoRa.print(", ");
  LoRa.print(dados_sender.gps_Velocidade);
  LoRa.print(", ");
  LoRa.print(dados_sender.gps_Direcao);
  LoRa.print("*");
  LoRa.println("/ ");
  LoRa.endPacket();
}
void enviaGPS() { //send GPS data
  Serial.print(dados_sender.millisegundos);
  Serial.print(", ");
  Serial.print(dados_sender.acel_x);
  Serial.print(", ");
  Serial.print(dados_sender.acel_y);
  Serial.print(", ");
  Serial.print(dados_sender.acel_z);
  Serial.print(", ");
  Serial.print(dados_sender.aceleracaoTotal);
  Serial.print(", ");
  Serial.print(dados_sender.bmeTemp);
  Serial.print(", ");
  Serial.print(dados_sender.bmePressure);
  Serial.print(", ");
  Serial.print(dados_sender.bmeHumidity);
  Serial.print(", ");
  Serial.print(dados_sender.bmeAltitude);
  Serial.print(", ");
  Serial.print(dados_sender.luz);
  Serial.print(", ");
  Serial.print(dados_sender.estado);
  Serial.print(", ");
  Serial.print(dados_sender.gps_Lat,6);
  Serial.print(", ");
  Serial.print(dados_sender.gps_Long,6);
  Serial.print(", ");
  Serial.print(dados_sender.gps_Alt);
  Serial.print(", ");
  Serial.print(dados_sender.gps_Velocidade);
  Serial.print(", ");
  Serial.print(dados_sender.gps_Direcao);
  Serial.println(", ");
}
void enviaSerial2() { //send data via Uart Serial --- send data for Raspberry Pi
  Serial2.print(dados_sender.millisegundos);
  Serial2.print(", ");
  Serial2.print(dados_sender.acel_x);
  Serial2.print(", ");
  Serial2.print(dados_sender.acel_y);
  Serial2.print(", ");
  Serial2.print(dados_sender.acel_z);
  Serial2.print(", ");
  Serial2.print(dados_sender.aceleracaoTotal);
  Serial2.print(", ");
  Serial2.print(dados_sender.bmeTemp);
  Serial2.print(", ");
  Serial2.print(dados_sender.bmePressure);
  Serial2.print(", ");
  Serial2.print(dados_sender.bmeHumidity);
  Serial2.print(", ");
  Serial2.print(dados_sender.bmeAltitude);
  Serial2.print(", ");
  Serial2.print(dados_sender.luz);
  Serial2.print(", ");
  Serial2.print(dados_sender.estado);
  Serial2.print(", ");
  Serial2.print(dados_sender.gps_Lat,6);
  Serial2.print(", ");
  Serial2.print(dados_sender.gps_Long,6);
  Serial2.print(", ");
  Serial2.print(dados_sender.gps_Alt);
  Serial2.print(", ");
  Serial2.print(dados_sender.gps_Velocidade);
  Serial2.print(", ");
  Serial2.print(dados_sender.gps_Direcao);
}
//--------execute voids--------
void readings() {//void to simulate/adapt the old loop void to multithreading
  int interval = 900;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    dados.millisegundos = millis(); //record time
    //execute sensores reading voids
    lerADXL();//read acel x, y, z values
    lerLuz();//read light values (luz)
    avaliaEstado();//avaluate current state (estado)
    lerBME();//read temperature, pressure and humidy values
    lerGPS();//read GPS data, basicaly everything that has gps. before it
    lerMPU();//read AccX, Y, Z; GyroX, Y, Z; Pitch, Roll and Yaw
    calculaAcceleracaoTotal();//calculate ACCEL using acel_x, y and z from data_sender struct
}

//--------Threads--------  !maybe change the name of the Threads now that the communication doesn't have to be 2 ways!
void HUB(void *parameter) { //main thread
  while(1){
    if(Busy == true){ //TODO: try to change the busy mecanic for vTaskSuspend to controll tasks and waste less process power
      Serial2.println("!Busy = True!");//DONE(17/04/24)
      vTaskResume(esp_turn_handle);
    } else {
      while(1){
        reading();
        envialora();
        enviaGPS();
        Serial2.println("ready"); //signal to rasp that esp is ready to receive commands
        serialEvent(); //analise Ser input
        if(stringComplete) {
          if(inputString.indexOf("on") > -1) { //equivelent of esp_turn
            Serial2.println("on"); //!repeat command ONLY FOR DEBUGG!
            Serial2.println("received"); //reception token
            Busy = true;
            vTaskResume(esp_turn_handle);//!Thread control with break and busy! 
            vTaskSuspend(HUB_handle);
          } else {
            Serial2.println("."); //TODO: create other case scenarios for quality of life reasons
            inputString = "";
            stringComplete = false;
          }
        }
      }
    }
  }
}

void esp_turn(void *parameter) { //(22/04/24)TODO: simplify this for the dear love of god
  while(1){                      //DONE(01/05/24): kinda...
    if(Busy == true || inputString.indexOf("on") > -1){ //(01/05/24)maybe try to turn this into somekind of list so like Serial.println(dados[i]) or smth
      Serial2.println(dados.millisegundos);             //Yo It's Flower from the future guess who wasen't able to simplify this because C++ isen't a dinamic language
      delay(wait_time);
      Serial2.println(dados.acel_x);
      delay(wait_time);
      Serial2.println(dados.acel_y);
      delay(wait_time);
      Serial2.println(dados.acel_z);
      delay(wait_time);
      Serial2.println(dados.aceleracaoTotal);
      delay(wait_time);
      Serial2.println(dados.bmeTemp);
      delay(wait_time);
      Serial2.println(dados.bmePressure);
      delay(wait_time);
      Serial2.println(dados.bmeHumidity);
      delay(wait_time);
      Serial2.println(dados.bmeAltitude);
      delay(wait_time);
      Serial2.println(dados.luz);
      delay(wait_time);
      Serial2.println(dados.estado);
      delay(wait_time);
      Serial2.println(dados.gps_Lat, 6);
      delay(wait_time);
      Serial2.println(dados.gps_Long, 6);
      delay(wait_time);
      Serial2.println(dados.gps_Alt);
      delay(wait_time);
      Serial2.println(dados.gps_Velocidade);
      delay(wait_time);
      Serial2.println(dados.gps_Direcao);      
      while(1){
        Serial2.println("sent_complete");
        serialEvent();
        if(inputString.indexOf("done") > -1){
          inputString = "";
          stringComplete = false;
          Busy = false; //!Thread controll with break and busy!
          vTaskResume(HUB_handle);
          break;
        }
      }
    }
  }
}

//control voids
void data_integrity_ver() { //default data set to later be updated with received data from RP
  int back_millisegundos = 0;
  int back_acel_x = 0;
  int back_acel_y = 0;
  int back_acel_z = 0;
  int back_aceleracaoTotal = 0;
  int back_bmeTemp = 0;
  int back_bmePressure = 0;
  int back_bmeHumidity = 0;
  int back_bmeAltitude = 0;
  int back_luz = 0;
  int back_estado = 0;
  int back_gps_Lat = 0;
  int back_gps_Long = 0;
  int back_gps_Alt = 0;
  int back_gps_Velocidade = 0;
  int back_gps_Direcao = 0;
  while(1){
    Serial.println("ser_update");
    serialEvent();
    if(stringComplete){
      if(inputString.indexOf("starting_comp") > -1){ //this makes me so confused like, why does this have to be so "unclean"
        Serial.println("ready_for_opp");             //I wish this code was cleaner but either I suck at explaining and searchning how to do something that I
        back_millisegundos = Serial.read();          //  know how to do in Python in C++ or this language is just dumb and has no var control
        if(back_millisegundos != dados.millisegundos) {dados.millisegundos = back_millisegundos;} //sorry for insulting C Levy if you ever read this repo ;P
        back_acel_x = Serial.read();
        if(back_acel_x != dados_send.acel_x) {dados_send.acel_x = back_acel_x;}
        back_acel_y = Serial.read();
        if(back_acel_y != dados_send.acel_y) {dados_send.acel_y = back_acel_y;}
        back_acel_z = Serial.read();
        if(back_acel_z != dados_send.acel_z) {dados_send.acel_z = back_acel_z;}
        back_aceleracaoTotal = Serial.read();
        if(back_aceleracaoTotal != dados_send.aceleracaoTotal) {dados_send.aceleracaoTotal = back_aceleracaoTotal;}
        back_bmeTemp = Serial.read();
        if(back_bmeTemp != dados_send.bmeTemp) {dados_send.bmeTemp = back_bmeTemp;}
        back_bmePressure = Serial.read();
        if(back_bmePressure != dados_send.bmePressure) {dados_send.bmePressure = back_bmePressure;}
        back_bmeHumidity = Serial.read();
        if(back_bmeHumidity != dados_send.bmeHumidity) {dados_send.bmeHumidity = back_bmeHumidity;}
        back_bmeAltitude = Serial.read();
        if(back_bmeAltitude != dados_send.bmeAltitude) {dados_send.bmeAltitude = back_bmeAltitude;}
        back_luz = Serial.read();
        if(back_luz != dados_send.luz) {dados_send.luz = back_luz;}
        back_estado = Serial.read();
        if(back_estado != dados_send.estado) {dados_send.estado = back_estado;}
        back_gps_Lat = Serial.read();
        if(back_gps_Lat != dados_send.gps_Lat) {dados_send.gps_Lat = back_gps_Lat;}
        back_gps_Long = Serial.read();
        if(back_gps_Long != dados_send.gps_Long) {dados_send.gps_Long = back_gps_Long;}
        back_gps_Alt = Serial.read();
        if(back_gps_Alt != dados_send.gps_Alt) {dados_send.gps_Alt = back_gps_Alt;}
        back_gps_Velocidade = Serial.read();
        if(back_gps_Velocidade != dados_send.gps_Velocidade) {dados_send.gps_Velocidade = back_gps_Velocidade;}
        back_gps_Direcao = Serial.read();
        if(back_gps_Direcao != dados_send.gps_Direcao) {dados_send.gps_Direcao = back_gps_Direcao;}   
        return;
      }
    }
  }
}

void sendStatus() { //like I think this is unnecessary but the last time I took this out it broke the hole code so...
  char buffer[50];
  inputValue = analogRead(analogPin);
  sprintf (buffer, "Analog in %d id %d", analogPin);
  Serial.println(buffer);
}

void serialEvent() { //Serial analiser !ONLY TAKES 8 BIT STUFF!
  while(Serial2.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar == '\n') { //just want to say THAT THIS GAVE ME SO MANY HEADHACKES BRO 
      stringComplete = true;
    }
  }
}

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
  data_integrity_ver(); //void to compare last RP read with ESP data to verefy with there was a reset
  for(int i = 0; i<5; i++){
    digitalWrite(outputPin, HIGH);
    delay(0.5);
    digitalWrite(outputPin, LOW);
    delay(2);
  }
  xTaskCreate(
    HUB,
    "HUB",
    1024,
    NULL,
    1,
    &HUB_handle);

  xTaskCreate(
    esp_turn,
    "esp_turn",
    1024,
    NULL,
    0,
    &esp_turn_handle);
}

void loop() {

}
