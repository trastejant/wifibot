#include "I2Cdev.h"
#include "MPU6050.h"
#include "pgmspace.h"
#include "SimpleDmp.h"
#include <Kalman.h>
#include <EEPROM.h>
 

#define D0 16 //DESTINADO A luces
#define D1 5 //DESTINADO A motores
#define D2 4 //DESTINADO A motores
#define D3 0 //DESTINADO A motores
#define D4 2 //DESTINADO A motores
#define D5 14 //DESTINADO A i2C
#define D6 12 //DESTINADO A i2C
#define D7 13 //DESTINADO A Interrupcion
#define D8 15 //DESTINADO A sensor de ultrasonidos

MPU6050 accelgyro;
SimpleDmp mpu;

int16_t ax, ay, az, gx,gy, gz;
uint8_t mpuIntStatus; 
uint16_t packetSize;
float yaw, pitch, roll;

bool dmpReady = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  //Interrupciones
  attachInterrupt(D8, dmpDataReady, RISING); //activamos la interrupcion
  mpuIntStatus = accelgyro.getIntStatus();
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;
  //packetSize = accelgyro.dmpGetFIFOPacketSize();

  //configuracion de los pines del motor
  pinMode(D1,OUTPUT);pinMode(D3,OUTPUT);
  pinMode(D2,OUTPUT);pinMode(D4,OUTPUT);  

  //inicializacion de la comunicacion serial e I2C
  Serial.begin(115200);
  Wire.begin(D6,D5);

  //iniializar dispositivo
  Serial.println("Inicializar dispotivos I2C");
  accelgyro.initialize();
  mpu.initDmp();

  //verificar la conexion
  Serial.println("Verificando la conexion...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 conexion establecida" : "MPU6050 conexion fallida");

  //calibracion del sensor MPU
  accelgyro.setXAccelOffset(-692);
  accelgyro.setYAccelOffset(-2290);
  accelgyro.setZAccelOffset(868);
  accelgyro.setXGyroOffset(57);
  accelgyro.setYGyroOffset(-20);
  accelgyro.setZGyroOffset(35);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (dmpReady){endereza();}

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
}

void endereza(){
  Serial.println("Recalculando posiciones");
  //accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); //Leemos acelerometro y giroscopo
  mpu.readMPU();
  mpu.getAngles(yaw, pitch, roll);
  Serial.print("Pitch, Yaw, Roll:" );Serial.print(pitch);Serial.print(" - ");Serial.print(yaw);Serial.print(" - ");Serial.println(roll);
  //Serial.print("Yaw:" );Serial.println(yaw);
  //Serial.print("Roll:" );Serial.println(roll);
  //delay(250);

  //if((pitch<-1)&&(pitch=>-0,9)){
  if(pitch<=-1){
    //avanza();
    retrocede();
    delay(10);
  }else if(pitch>=-0,95){
    //retrocede();
    avanza();
    delay(10);  
  }
  
}

  void retrocede(){
  digitalWrite(D1,HIGH);
  digitalWrite(D3,LOW);

  digitalWrite(D2,HIGH);
  digitalWrite(D4,LOW);
}

void avanza(){
  digitalWrite(D1,HIGH);
  digitalWrite(D3,HIGH);
  
  digitalWrite(D2,HIGH);
  digitalWrite(D4,HIGH);
}

void para(){
  digitalWrite(D1,LOW);
  digitalWrite(D3,LOW);
  
  digitalWrite(D2,LOW);
  digitalWrite(D4,LOW);
  }
