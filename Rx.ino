/* Receiver code for the Arduino Radio control with PWM output
 *  
 *  THIS ONLY WORKS WITH ATMEGA328p registers!!!!
 *  It gives a nice PWM output on pins D2, D3, D4, D5, D6 and D7. Still working on it...
 *  
 *  Install the NRF24 library to your IDE
 *  Import the servo library as well
 * Upload this code to the Arduino UNO
 * Connect a NRF24 module to it:
 
    Module // Arduino UNO
    
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12

This code receive 6 channels and create a PWM output for each one on D2, D3, D4, D5, D6 and D7
Please, like share and subscribe : https://www.youtube.com/c/ELECTRONOOBS
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include <EEPROM.h>

#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINT(x)    Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x) 
#endif

//Define widths
int pwm_width_2 = 0;
int pwm_width_3 = 0;
int pwm_width_4 = 0;
int pwm_width_5 = 0;
int pwm_width_6 = 0;
int pwm_width_7 = 0;

Servo PWM2;
Servo PWM3;
Servo PWM4;
Servo PWM5;
Servo PWM6;
Servo PWM7;

                    //We could use up to 32 channels
struct MyData {
byte throttle;      //We define each byte of data input, in this case just 6 channels
byte yaw;
byte pitch;
byte roll;
byte AUX1;
byte AUX2;
};
MyData data;

//variable NRF24L01

const uint64_t pipes[2] = {0xE8E8F0F0E1LL,0xE8E8F0F0E2LL}; //IMPORTANT: The same as in the receiver
RF24 radio(9, 10); 

//Variables medición de batería

int BatTemp=0;
int loopbat=10;
byte Bateria=0;

//vector para almacenamiento en EEPROM
byte datosEEPROM[]={0,127,127,127};//inicializo con los valores que debería cargar en caso de que no haya nada, pero si hay algo los sobreescribo ni bien arranco
int i; //variable para recorrer vector y leer/escribir datos

void resetData()
{
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = datosEEPROM[0];
data.yaw = datosEEPROM[1];
data.pitch = datosEEPROM[2];
data.roll = datosEEPROM[3];
data.AUX1 = 0;
data.AUX2 = 0;
}

/**************************************************/

void setup()
{
  //Set the pins for each PWM signal
  PWM2.attach(2);
  PWM3.attach(3);
  PWM4.attach(4);
  PWM5.attach(5);
  PWM6.attach(6);
  PWM7.attach(7);

  analogReference(INTERNAL); // use the internal ~1.1volt reference
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  //Configure the NRF24 module
  
  resetData();
  radio.begin();
  //radio.setAutoAck(false);
  
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);  
  
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  
  //we start the radio comunication
  radio.startListening();

  //valores por defecto EEPROM
  if (EEPROM.read(511)!=0) //si lo que haya en posición 511 (última) es distinto de 0 es que nunca escribí un dato válido, entonces inicializo
    {
     for( i=0 ; i<4 ; i++)
     {
      EEPROM.write(i,datosEEPROM[i]);  // dirección - dato
     }
    EEPROM.write(511,0);//y pongo en 0 la última posición para no pisar los valores la próxima vez que entre    
    }   
  else //si 511=0 es que tengo los datos que necesito en memoria, entonces los leo al vector
    {
    for( i=0 ; i<4 ; i++)
     {
     datosEEPROM[i] = EEPROM.read(i);  
     }
  }
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{
  //****
  byte pipeNo, gotByte;
  BatTemp+=analogRead(A0);
  loopbat--;
  if(loopbat==0)
    {
    loopbat=10;
    Bateria=(BatTemp/10)-768; //promedio 10 lecturas del ADC (no bloqueante) y después le resto 768 para que entre en 1 byte sin comprimir informacion
    DEBUG_PRINT(BatTemp/10);
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN(Bateria);
    BatTemp=0;
    }
  while( radio.available(&pipeNo)){              // Read all available payloads
      radio.read(&data, sizeof(MyData));                  
      //Bateria=analogRead(A0);                                             // Since this is a call-response. Respond directly with an ack payload.
      //gotByte += 1;                                // Ack payloads are much more efficient than switching to transmit mode to respond to a call
      radio.writeAckPayload(pipeNo,&Bateria, 1 );  // This can be commented out to send empty payloads.
      //DEBUG_PRINT(F("Loaded next response "));
      //DEBUG_PRINTLN(Bateria);  
      lastRecvTime = millis(); //here we receive the data
   }

}

void writeDefault()
{
//en la variable datosEEPROM siempre tengo lo que tengo que pasar a los servos en caso de pérdida de señal
datosEEPROM[0]=data.throttle;
datosEEPROM[1]=data.yaw;
datosEEPROM[2]=data.pitch;
datosEEPROM[3]=data.roll;

for( i=0 ; i<4 ; i++) //guardo la nueva posición por defecto en la EEPROM
     {
      EEPROM.write(i,datosEEPROM[i]);  // dirección - dato
     }
}

/**************************************************/

void loop()
{
recvData();
unsigned long now = millis();

//Here we check if we've lost signal, if we did we reset the values 

if ( now - lastRecvTime > 1000 ) // Signal lost?
  {
  resetData();
  }

if(now - lastRecvTime > 120000) //si perdí la señal por 2 minutos pongo los valores para que caiga
  {//los pongo directo acá por que si bien también va a entrar el if anterior y va a poner los valores por defecto este los va a sobreescribir y la idea es que acá no entre nunca
    //la pérdida de procesamiento va a ser menor que revisando en cada vuelta que sea mayor a 1000 y menor a 120.000
  data.throttle=0; //apagar motor
  data.yaw = 0; //todo para la izquierda con la cola
  data.pitch = 255; //todo para abajo con elevador
  data.roll = 127;
    
  }

//verifico si recibí la orden de escribir valores por defecto nuevos en EEPROM
if(data.AUX2==255)
  writeDefault();

pwm_width_2 = map(data.throttle, 0, 255, 1000, 2000);     //PWM value on digital pin D2
pwm_width_3 = map(data.yaw,      0, 255, 1000, 2000);     //PWM value on digital pin D3
pwm_width_4 = map(data.pitch,    0, 255, 1000, 2000);     //PWM value on digital pin D4
pwm_width_5 = map(data.roll,     0, 255, 1000, 2000);     //PWM value on digital pin D5
pwm_width_6 = map(data.AUX1,     0, 255, 1000, 2000);     //PWM value on digital pin D6
pwm_width_7 = map(data.AUX2,     0, 255, 1000, 2000);     //PWM value on digital pin D7


//Now we write the PWM signal using the servo function
PWM2.writeMicroseconds(pwm_width_2);
PWM3.writeMicroseconds(pwm_width_3);
PWM4.writeMicroseconds(pwm_width_4);
PWM5.writeMicroseconds(pwm_width_5);
PWM6.writeMicroseconds(pwm_width_6);
PWM7.writeMicroseconds(pwm_width_7);


}//Loop end
/**************************************************/
