
#include <LiquidCrystal.h>
#include <EEPROM.h>

//comunicación
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*
 * Versión 1.8 - Agregado guardar en EEPROM posición por defecto ante pérdida de señal
 * 
 */

 /*
  * Librerías EEPROM
  * Cuando el canal "AUX2" se pone en 255 detecta que esa es la posición que debe guardar, copia valores en vector temporal? y llama a función
  * que escriba EEPROM
  * Por defecto arranca leyendo la EEPROM y si no hay un valor válido guarda todo centrado
  */
//Librerías Joystick
#include <PS2X_lib.h>  //for v1.6
PS2X ps2x; // create PS2 Controller Class

//#define DEBUG //descomentar esta linea para activar los serialprint

#ifdef DEBUG
 #define DEBUG_PRINT(x)    Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x) 
#endif

#define CHANNELS 4
#define SWITCHES 2
#define MODELOS 5
#define CANALTH 0

#define swTC 0
#define swDR 1

#define eepromModeloId 0;

//variables joystick
int js_error = 0; 
byte js_type = 0;
byte vibrate = 0;//Variable para vibración del joystick. No se usa en esta versión

//variables comunicación
/*Create a unique pipe out. The receiver has to 
  wear the same unique code*/
  
const uint64_t pipes[2] = {0xE8E8F0F0E1LL,0xE8E8F0F0E2LL}; //IMPORTANT: The same as in the receiver
//2 pipes. Una para emitir los datos y otra para recibir el ack con la información de telemetría
RF24 radio(9, 10); // select  CSN  pin

uint16_t g_input[CHANNELS];

// canales del TX
int menuActual=0;

unsigned char modelo_actual=0;  // id del modelo actual
unsigned char num_menus=5; // numOpciones-1
unsigned char submenu_actual=0;  // para cuando este dentro del menu
unsigned char servo_reversa[MODELOS];
unsigned char servo_reversa_cambio=0;
unsigned char estado_display=0;  // variable para el refresco del display

unsigned char canal_cambio=0;  // variable para seleccion de canal en los submenus

unsigned char canal_cambio_dr=0;
unsigned char dual_rate_low[MODELOS][CHANNELS];
unsigned char dual_rate_hi[MODELOS][CHANNELS];

// valores varios
int potenciometros[CHANNELS];  // lectura de potenciometros
unsigned char sw_status=0x00;    //0:TC 1:DR  2:EXPO 
int potMin = 0;
int potMax = 255;

byte Bateria=0; //esta variable es la que uso para recibir el dato
int BateriaLocal=0; //para leer el ADC local
bool bat=0; //indica qué batería hay que mostrar. 0 es remota
char Str[6]=" ";//en esta lo guardo formateado para mostrar
float BatDec=0;//esta es para pasar el valor de lectura a una tensión 

int Ping=0;

int esperar=0;

//variables de estado para entradas
bool EstadoUp=0;
bool EstadoDown=0;
bool EstadoLeft=0;
bool EstadoRight=0;
bool EstadoSelect=0;
bool EstadoL2=0; //Throtle Cut
bool EstadoTriangulo=0; //si aprieto triángulo el receptor guarda por defecto ante pérdida de señal los valores actuales
                        //la idea es poner el avión a dar vueltas y apretarlo, cosa de que después haga solito lo mismo
bool EstadoTC=1;
bool EstadoR2=0; //Dual rates
bool EstadoR1=0; //Bateria local o remota
bool EstadoDR=1;
// LCD
LiquidCrystal lcd(2, 3, 17, 16, 15, 14); //RS-A5/EN-A4/D4-A3/D5-A2/D6-A1/D7-A0

//***********************************************************************ESTRUCTURAS**********************************************************************


// mensajes del menu
char msgs[6][17] = {
  "Menu        ",
  "1.Servo Dir.", 
  "2.Dual Rates", 
  "3.E.P.A.    ", 
  "4.Guardar   ", 
  "5.Modelo    " };

byte lcdChar[][8] = { 
  {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x1F},
  
  {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x1F,
  0x00},
  
  {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x1F,
  0x00,
  0x00},
  
  {
  0x00,
  0x00,
  0x00,
  0x00,
  0x1F,
  0x00,
  0x00,
  0x00},
  
  {
  0x00,
  0x00,
  0x00,
  0x1F,
  0x00,
  0x00,
  0x00,
  0x00},
  
  {
  0x00,
  0x00,
  0x1F,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00},
  
  {
  0x00,
  0x1F,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00},
  
  {
  0x1F,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00}
  
} ;
// /LCD

//Estrucura comunicación
// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data;


//***********************************************************************FUNCIONES**********************************************************************

void resetData() 
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.
    
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

int ajusteValor(int maximo, int minimo, int valor) {
  int salida;
  float divisor;
  divisor=255.0/(maximo-minimo);
  salida=valor/divisor;
  return salida;
}

int ajusteDR(int valor, int dr, int canal) {
  int salida,temp;
  float mult;
  int canalth=CANALTH;
  
  if (canal==canalth) {
    // dual rate del total
    mult=dr/100.0;
    salida=valor*mult;  
  } else {
     // dual rate de los extremos
     temp=valor-127;
     mult=dr/100.0;
     salida=(temp*mult)+127;
     
  }
  
  return salida;
}

void leePotenciometros() {
  
  uint8_t i;
  
  ps2x.read_gamepad(false, vibrate);//En vez de leer lo potes directo leo el joystick

  potenciometros[0]=ps2x.Analog(PSS_LY); //Temporal para probar usar los pines analogicos para el display
  potenciometros[1]=ps2x.Analog(PSS_LX);  
  potenciometros[2]=ps2x.Analog(PSS_RY); 
  potenciometros[3]=ps2x.Analog(PSS_RX); 
  
  // leemos los switches//***********************************************

  if(ps2x.Button(PSB_L2)&&(!EstadoL2)) //L2 cortar propulsión
    EstadoL2=1;
  if(!ps2x.Button(PSB_L2)&&(EstadoL2))
    {
    EstadoTC=!EstadoTC;
    bitWrite(sw_status,0,EstadoTC);
    EstadoL2=0;
    }

  if(ps2x.Button(PSB_R2)&&(!EstadoR2))//Dual rates
    EstadoR2=1;
  if(!ps2x.Button(PSB_R2)&&(EstadoR2))
    {
    EstadoDR=!EstadoDR;
    bitWrite(sw_status,1,EstadoDR);
    EstadoR2=0;
    }

  if(ps2x.Button(PSB_R1)&&(!EstadoR1))//Batería local - remota
    EstadoR1=1;
  if(!ps2x.Button(PSB_R1)&&(EstadoR1))
    {
    bat=!bat;
    EstadoR1=0;
    }

  if(ps2x.Button(PSB_GREEN)&&(!EstadoTriangulo))//guardar valores por defecto  
    EstadoTriangulo=1;
  if(!ps2x.Button(PSB_GREEN)&&(EstadoTriangulo))
    {
    //Hacer lo necesario
    data.AUX2     = 255; //lo mando como 255 acá y lo reseteo después de enviar
    EstadoTriangulo=0;
    }
    
  // activamos el TC cuando se cierra el switch
  if (bitRead(sw_status,0)==0) {
    // TC activado
    potenciometros[CANALTH]=255;
  } 
  
  
  // reversa de servos
  for (i = 0;  i < CHANNELS; ++i){
      if (bitRead(servo_reversa[modelo_actual], i)) {
          potenciometros[i]=255-potenciometros[i];
      }
  }
  
  // dual rates
  for (i = 0;  i < CHANNELS; ++i){
    // activamos el DR cuando se cierra el switch
    if (bitRead(sw_status,1)==0) {
       // low rates
       potenciometros[i]=ajusteDR(potenciometros[i],dual_rate_low[modelo_actual][i],i);
       
    } else {
       // hi rates 
       potenciometros[i]=ajusteDR(potenciometros[i],dual_rate_hi[modelo_actual][i],i);
    }  
  }
  
  data.throttle = potenciometros[0];
  data.yaw      = potenciometros[1];
  data.pitch    = potenciometros[2];
  data.roll     = potenciometros[3];
  
  
	//NRF
  
  radio.stopListening();
  unsigned long time = micros();
  //***********
  if ( radio.write(&data, sizeof(MyData)))
      {                         // Send the counter variable to the other radio 
      digitalWrite(8, LOW);     //apago el led de no se pudo enviar
      if(!radio.available())
        {                             // If nothing in the buffer, we got an ack but it is blank
        DEBUG_PRINT(F("Got blank response. round-trip delay: "));
        DEBUG_PRINT(micros()-time);
        DEBUG_PRINTLN(F(" microseconds"));     
        }
      else
        {      
        while(radio.available())
          {                      // If an ack with payload was received
          radio.read( &Bateria, 1 );                  // Read it, and display the response time
          unsigned long timer = micros();
          //Info de telemetría y tiempo de ida y vuelta
          Ping=timer-time;
                
          DEBUG_PRINT(F("Got response "));
          DEBUG_PRINT(Bateria);
          DEBUG_PRINT(F(" round-trip delay: "));
          DEBUG_PRINT(timer-time);
          DEBUG_PRINTLN(F(" microseconds"));
          
          }
        }
    
      }
  else
    {
    DEBUG_PRINTLN(F("Sending failed.")); 
    digitalWrite(8, HIGH);//prendo el led de no se pudo enviar. Infrma de pérdida de conexión entre emisor y receptor
    }          // If no ack response, sending failed
    ///*******************
  
  //bool ok=radio.write(&data, sizeof(MyData));
  //if(ok)
    //DEBUG_PRINTLN("Datos enviados");

//necesito mandar un 255 si aprete el boton de escribir valores por defecto, pero que después de mandar vuelva a cero, así que lo reseteo después de mandar. 

    data.AUX1     = 0; //Esto por ahora lo dejo en cero. Para agregar más datos meterlos aca
    data.AUX2     = 0;
}

int porcentajePotenciometro(float valor, int minimo, float maximo) {
    double temp;
    temp=100*valor/maximo;
    return (int)temp;
}

unsigned char potenciometroBar(float valor, float maximo) {
   double temp;
    temp=7*valor/maximo;
   return (int)temp; 
}

void setup() {
  #ifdef DEBUG
  Serial.begin(57600);//Esto es para debug de joystick
  #endif
  pinMode(8, OUTPUT);
  int i,j;
  
  //init de valores
   
  // servos
  for (i=0;i<=MODELOS-1;i++) {
    servo_reversa[i]=0;
  }
  // dual rates
  for (i=0;i<=MODELOS-1;i++) {
    for (j=0;j<=(CHANNELS-1);j++) {
       dual_rate_low[i][j]=75;
       dual_rate_hi[i][j]=100;
    }
  }
  
  // si eeprom(511)=!0, reseteamos todos los datos de las memorias

  if (EEPROM.read(511)!=0) {
     EEPROM.write(0,0);  // modelo 0
    // guardamos el modelo actual
    for (modelo_actual=0;modelo_actual<=MODELOS;modelo_actual++) {
      EEPROM.write((modelo_actual*16)+1,servo_reversa[modelo_actual]);
      // guardamos los datos del dual rate
      for (i=0;i<=CHANNELS;i++) {
         EEPROM.write((modelo_actual*16)+((2*i)+2),dual_rate_low[modelo_actual][i]);
         EEPROM.write((modelo_actual*16)+((2*i)+3),dual_rate_hi[modelo_actual][i]);
         
      }
    }
    EEPROM.write(511,0);    
  }   
  
  
  // leemos los datos desde la eeprom
    modelo_actual=EEPROM.read(0);
    // leemos el dato de servo reversa
       servo_reversa[modelo_actual]=EEPROM.read((modelo_actual*16)+1);
       // leemos los datos del dual rate
       // guardamos los datos del dual rate
       for (i=0;i<=CHANNELS;i++) {
         dual_rate_low[modelo_actual][i]=EEPROM.read((modelo_actual*16)+((2*i)+2));
         dual_rate_hi[modelo_actual][i]=EEPROM.read((modelo_actual*16)+((2*i)+3));
         
       }
       
   // si eeprom(511)=255, reseteamos todos los datos de las memorias

  if (EEPROM.read(511)==255) {
   
  }   
  
  for (uint8_t i = 0;  i < CHANNELS-1; ++i)
	{
		
		// fill input buffer, convert raw values to microseconds
		g_input[i] = map(potenciometros[i], 0, 255, 1000, 2000);
	}
 
  // lcd
  
  lcd.createChar(1, lcdChar[0]);
  lcd.createChar(2, lcdChar[1]);
  lcd.createChar(3, lcdChar[2]);
  lcd.createChar(4, lcdChar[3]);
  lcd.createChar(5, lcdChar[4]);
  lcd.createChar(6, lcdChar[5]);
  lcd.createChar(7, lcdChar[6]);
  lcd.createChar(8, lcdChar[7]);
  
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Engendro v.1.9");
  lcd.setCursor(0,1);
  lcd.print("Momo 2018  ");
   
  delay(1000);
  lcd.clear();

  
  //Config joystick
  js_error = ps2x.config_gamepad(4,5,6,7, true, true);   //setup pins and settings:  GamePad(clock(blue), command(orange), attention(yellow), data(brown), Pressures?, Rumble?) check for js_error

  //**********************************Errores control
  
  if(js_error == 0){
    DEBUG_PRINTLN("Found Controller, configured successful"); 
    }
  else if(js_error == 1)
   DEBUG_PRINTLN("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(js_error == 2)
   DEBUG_PRINTLN("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(js_error == 3)
   DEBUG_PRINTLN("Controller refusing to enter Pressures mode, may not support it. ");
  
  
  js_type = ps2x.readType(); 
     switch(js_type) {
       case 0:
        DEBUG_PRINTLN("Unknown Controller type");
       break;
       case 1:
        DEBUG_PRINTLN("DualShock Controller Found");
       break;
       case 2:
         DEBUG_PRINTLN("GuitarHero Controller Found");
       break;
     }

   //fin cosas joystick


  //NRF
  //Start everything up
  radio.begin();
  //radio.setAutoAck(false);
  
  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads
  
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);

  radio.startListening();                       // Start listening

  radio.writeAckPayload(1,&Bateria,1);
  
  resetData();
}



void loop() {
  unsigned char tecla;//***********************************************??????? xq declara dentro del loop?
  unsigned char i;
  
  leePotenciometros();

  // pantalla principal
  if (submenu_actual==0) {
    lcd.setCursor(0, 1); 
    lcd.print(msgs[menuActual]);
    // sliders de los potenciometros

    //bateria y tiempo
    
    if(!esperar)
      {
      //bat
      lcd.setCursor(14, 0);

      if(!bat)//si es cero muestro la del avion
        {
        BatDec=(Bateria+768)*0.0121504; //(1.0814/(1023.0*0.087)) acá hay que calibrar según tensión de referecia y divisor resistivo
                                        //sumo 768 ya que el ack es limitado y de comprimir el valor para que entre reduciría la resolución
                                        //cuando en la práctica sólo necesito medir un rango limitado de carga
        lcd.write("+");
        }
      else //local
        {
        BatDec=(analogRead(A5))*0.0040884;//(3.302/(1023.0*0.7895));
        
        lcd.write("-");
        }

      dtostrf(BatDec, 5, 2, Str);
      lcd.print(Str);
      //tiempo
      dtostrf(Ping, 6, 0, Str);
      lcd.setCursor(14,1);
      lcd.print(Str);
      //reiniciar contador
      esperar=20;
      }
    
    esperar--;
    // modelo
    lcd.setCursor(0,0);
    lcd.write("M");
    lcd.print(modelo_actual+1);
    
    // indicador throtle cut
    lcd.setCursor(3,0);
    if (bitRead(sw_status,0)==0) {
      lcd.write("TC");
    } else {
      lcd.write("  ");
    }
    
    // indicador dual rate
    lcd.setCursor(6,0);
    if (bitRead(sw_status,1)==0) {
      lcd.write("LO");
    } else {
      lcd.write("HI");
    }
    
    lcd.setCursor(9,0);
    lcd.write(potenciometroBar(potenciometros[0],255)+1);
    lcd.write(potenciometroBar(potenciometros[1],255)+1);
    lcd.write(potenciometroBar(potenciometros[2],255)+1);
    lcd.write(potenciometroBar(potenciometros[3],255)+1);

    tecla=lee_teclado();
    if (tecla!=-1) {
      if (tecla==1 && menuActual<num_menus) menuActual++;
      if (tecla==2 && menuActual>0) menuActual--;
      if (tecla==4) { 
        lcd.clear(); 
        canal_cambio=0;
        submenu_actual=menuActual; 
      }

    }

  }

  // servo reversa
  if (submenu_actual==1) {
    if (estado_display==0) {
      lcd.setCursor(0, 0);  //line=2, x=0
      lcd.print(msgs[submenu_actual]);
      lcd.setCursor(0, 1); 
      for (i=0;i<=(CHANNELS-1);i++) {
        /*
        switch (i) {
        case 0:  
          lcd.print("A:");
          break;
        case 1:  
          lcd.print("E:");
          break;
        case 2:  
          lcd.print("T:");
          break;
        case 3:  
          lcd.print("R:");
          break;
        default: 
          lcd.print(i+1);
          lcd.print(":");
          break;
        }
        */
        lcd.print(i+1);
        lcd.print(":");
        if (bitRead(servo_reversa[modelo_actual],i)) {
          lcd.print("R ");
        } 
        else {
          lcd.print("N ");
        }
        
      }

    }
    estado_display=1;
    lcd.setCursor((servo_reversa_cambio*4)+2,1);
    lcd.blink();
        
    tecla=lee_teclado();
    if (tecla!=-1) {
      if (tecla==0 && servo_reversa_cambio<(CHANNELS-1)) { servo_reversa_cambio++; }
      if (tecla==3 && servo_reversa_cambio>0) { servo_reversa_cambio--; }
      
      if (tecla==1) {
         bitWrite(servo_reversa[modelo_actual], servo_reversa_cambio, 1);
         estado_display=0;
      }
      if (tecla==2) {
         bitWrite(servo_reversa[modelo_actual], servo_reversa_cambio, 0);
         estado_display=0;
      }
      
      if (tecla==4) { 
        lcd.clear(); 
        estado_display=0; 
        submenu_actual=0; 
      }

    }
  }

  // dual rates
  if (submenu_actual==2) {
     if (estado_display==0) {
        lcd.clear();
        lcd.setCursor(0, 0);  //line=2, x=0
        lcd.print(msgs[submenu_actual]);
        lcd.setCursor(0, 1); 
        lcd.print("CH:");
        lcd.print(canal_cambio+1);
        lcd.setCursor(5, 1); 
        lcd.print("L:");
        lcd.print(dual_rate_low[modelo_actual][canal_cambio]);
        
        lcd.setCursor(11, 1); 
        lcd.print("H:");
        lcd.print(dual_rate_hi[modelo_actual][canal_cambio]);
        
        switch (canal_cambio_dr) {
           case 0: lcd.setCursor(3, 1); break;
           case 1: lcd.setCursor(7, 1); break;
           case 2: lcd.setCursor(13, 1); break;
        }
        
        lcd.blink();
        estado_display=1;
      
        
     }
     tecla=lee_teclado();
     if (tecla!=-1) {
      
      // cambio de canal
      if (tecla==1 && canal_cambio<(CHANNELS-1) && canal_cambio_dr==0) {
        canal_cambio++;
        estado_display=0;  
      }
      if (tecla==2 && canal_cambio>0 && canal_cambio_dr==0) {
        canal_cambio--;  
        estado_display=0;
      }
      
      // cambio de valor en el dual rate actual
      if (tecla==1 && canal_cambio_dr==1 && dual_rate_low[modelo_actual][canal_cambio]<100) {
          dual_rate_low[modelo_actual][canal_cambio]++;
          estado_display=0;
      }
      if (tecla==1 && canal_cambio_dr==2 && dual_rate_hi[modelo_actual][canal_cambio]<100) {
          dual_rate_hi[modelo_actual][canal_cambio]++;
          estado_display=0;
      }
      if (tecla==2 && canal_cambio_dr==1 && dual_rate_low[modelo_actual][canal_cambio]>0) {
          dual_rate_low[modelo_actual][canal_cambio]--;
          estado_display=0;
      }
      if (tecla==2 && canal_cambio_dr==2 && dual_rate_hi[modelo_actual][canal_cambio]>0) {
          dual_rate_hi[modelo_actual][canal_cambio]--;
          estado_display=0;
      }
      
      
      
      // cambio de selector de dual rate para el canal actual (H/L)
      if (tecla==0 && canal_cambio_dr<2) {
        canal_cambio_dr++;
        estado_display=0;  
      }
      if (tecla==3 && canal_cambio_dr>0) {
        canal_cambio_dr--;
        estado_display=0;  
      }
      
      if (tecla==4) { 
        lcd.clear(); 
        estado_display=0; 
        submenu_actual=0; 
      }

    }
     
  }
  
  
  // guardar datos en modelo actual
  if (submenu_actual==4) {
    // guardamos el modelo actual
    EEPROM.write(0,modelo_actual);
    // guardamos los datos de la direccion de los servos
    EEPROM.write((modelo_actual*16)+1,servo_reversa[modelo_actual]);
    // guardamos los datos del dual rate
    for (i=0;i<=CHANNELS;i++) {
       EEPROM.write((modelo_actual*16)+((2*i)+2),dual_rate_low[modelo_actual][i]);
       EEPROM.write((modelo_actual*16)+((2*i)+3),dual_rate_hi[modelo_actual][i]);
       
    }
    
    // mensaje y regreso
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Modelo: ");
    lcd.print(modelo_actual+1);
    lcd.setCursor(0, 1);
    lcd.print("grabado");
    delay(600);
    lcd.clear();
    estado_display=0; 
    submenu_actual=0;
    
  }
  
  // modelo
  if (submenu_actual==5) {
    if (estado_display==0) {
        lcd.clear();
        lcd.setCursor(0, 0);  //line=2, x=0
        lcd.print(msgs[submenu_actual]);
        lcd.setCursor(0, 1); 
        lcd.print("Modelo: ");
        lcd.print(modelo_actual+1);
        lcd.blink();
        estado_display=1;
      
        
     }  
     tecla=lee_teclado();
     if (tecla!=-1) {
        if (tecla==1 && modelo_actual<=MODELOS) {
           modelo_actual++;
           
        }
        if (tecla==2 && modelo_actual>0) {
           modelo_actual--;
        }
        lcd.setCursor(0, 1); 
        lcd.print("Modelo: ");
        lcd.print(modelo_actual+1);
        
     }
     
     if (tecla==4) { 
       // leemos los datos desde la eeprom
       
       EEPROM.write(0,modelo_actual);
       // leemos el dato de servo reversa
       servo_reversa[modelo_actual]=EEPROM.read((modelo_actual*16)+1);
       // leemos los datos del dual rate
       // guardamos los datos del dual rate
       for (i=0;i<=CHANNELS;i++) {
         dual_rate_low[modelo_actual][i]=EEPROM.read((modelo_actual*16)+((2*i)+2));
         dual_rate_hi[modelo_actual][i]=EEPROM.read((modelo_actual*16)+((2*i)+3));
         
       }
     
        lcd.clear(); 
        estado_display=0; 
        submenu_actual=0; 
     }
       
  }
}

unsigned char lee_teclado() {
  ps2x.read_gamepad(false, vibrate);

  unsigned char salida=-1;

  if(ps2x.Button(PSB_PAD_RIGHT)&&(!EstadoRight))
    EstadoRight=1;
  if(!ps2x.Button(PSB_PAD_RIGHT)&&(EstadoRight))
    {
    salida=0;
    EstadoRight=0;
    }
  
  if(ps2x.Button(PSB_PAD_UP)&&(!EstadoUp))
    EstadoUp=1;
  if(!ps2x.Button(PSB_PAD_UP)&&(EstadoUp))
    {
    salida=1;
    EstadoUp=0;
    }

  if(ps2x.Button(PSB_PAD_DOWN)&&(!EstadoDown))
    EstadoDown=1;
  if(!ps2x.Button(PSB_PAD_DOWN)&&(EstadoDown))
    {
    salida=2;
    EstadoDown=0;
    }
    
  if(ps2x.Button(PSB_PAD_LEFT)&&(!EstadoLeft))
    EstadoLeft=1;
  if(!ps2x.Button(PSB_PAD_LEFT)&&(EstadoLeft))
    {
    salida=3;
    EstadoLeft=0;
    }
    
  if(ps2x.Button(PSB_BLUE)&&(!EstadoSelect))
    EstadoSelect=1;
  if(!ps2x.Button(PSB_BLUE)&&(EstadoSelect))
    {
    salida=4;
    EstadoSelect=0;
    }

  //delay(25);
  return salida;
  
}


