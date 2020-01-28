/*
   Experimental prototype for measuring stations
   =============================================

   COMPANY: ITER
   AUTHOR: Carolina Montañez, José Barrancos, Vidal Domínguez 
   EMAIL: amontanez@iter.es
   YEAR : 2019
   VERSION : 1.0 - Para Licor 850 y 820 con SDcard
   NOTA IMPORTATE,ANTES DE SUBIR CÓDIGO:
   WORST: SoftwareSerial is NOT RECOMMENDED, because it disables interrupts for long periods of time.
   This can interfere with other parts of your sketch, or with other libraries.
   It cannot transmit and receive at the same time, and your sketch can only receive
   from one SoftwareSerial instance at time
   https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Installing.md#2-choose-a-serial-port

  <li820>  <data>   <celltemp>5.14537e1</celltemp>   <cellpres>9.79897e1</cellpres>   <co2>2.87624e3</co2>
                  <co2abs>2.6884908e-1</co2abs>    <ivolt>1.4199829e1</ivolt>    <raw>3787568,3639155</raw>
         </data>
  </li820>

  <li850>  <data>   <celltemp>5.14537e1</celltemp>   <cellpres>9.79897e1</cellpres>   <co2>2.87624e3</co2>
                  <co2abs>2.6884908e-1</co2abs>   <h2o>3.00564e1</h2o>  <h2oabs>1.4000678e-1</h2oabs>
                  <h2odewpoint>2.37214e1</h2odewpoint>   <ivolt>1.2252674e1</ivolt>
                 <raw>    <co2>3503854</co2>   <co2ref>5519850</co2ref>   <h2o>2540376</h2o>  <h2oref>3526721</h2oref>  </raw>
                <flowrate>5.3951334e-2</flowrate>
         </data>
  </li850>
*/
#include <Arduino.h>
//Libs
//#include <DHT.h>                //DHT dht(HUMPIN, DHTTYPE);
//#include <DHT_U.h>              //DHT_Unified dhtu(HUMPIN, DHTTYPE);
#include <SimpleDHT.h> //Se usa simpleDHT22 porque la libreria DHT tiene un delay de 270ms

//#include <SD.h> // PARA SD
#include <SPI.h>

#include <ArduinoJson.h>

#include <Wire.h> //Conversores AD
#include <Adafruit_ADS1015.h>

#include "ST7565.h" //Display
#include <stdio.h>

#include <NMEAGPS.h> //GPS
#include <Arduino.h>
#include <GPSport.h>

#include <LinearRegression.h>        //libreria para calculos matematicos
#include <Average.h>
#include "CircularBuffer.h"

#include "GFButton.h" //Para botones
#include "MathHelpers.h"  // conversion de float a notacion cientifica

//----------------------------------------------------------------------------------------------------------------------------------------------

#define USB_PORT Serial
#ifndef GPSport_h
#define GPSport_h
#define gpsPort Serial1
#define GPS_PORT_NAME "Serial"
#endif
#define LICOR_PORT Serial3
#define BT_PORT Serial2

//Pins
#define LEDPIN 13          // activity led
#define RELE_BOMBA 2       //Bomba aspiracion
#define BOMBA_INYECCION 3  //Bomba de inyeccion
#define TIEMPO_TEST_B 1000 // tiempo de test de bomba
// #define ULTPIN_TRIG 25    //  TRIG y ECHO ultrasonido
// #define ULTPIN_ECHO 26
//  TO-DO cambiar por
#define ULTPIN_TRIG 25
#define ULTPIN_ECHO 26
#define HUMPIN 23
#define INICIO_B 4 //botones
#define ACUMULAR_B 5
#define FIN_B 6

#define SDA0_PIN 17 //SDA Y SCL para conversores A/D.  Solo para TEENSY 3.2
#define SCL0_PIN 16

//Canales analogicos del ads 0
#define LICOR_CO2_ANA 0 // LICOR h20 analogico
#define LICOR_H2O_ANA 1 // LICOR h20 analogico
#define CAUDALIMETRO 2  // caudalimentro conectado a PIN A2 DAC0
#define PRES_BAR 3      // Sensor de presión barométrica conectado a PIN A3 DAC1
//Canales analógicos del ads 1
#define A10 0
#define A11 1
#define A12 2
#define A13 3

/*
   DHT dht(HUMPIN, DHTTYPE);
   DHT_Unified dhtu(HUMPIN, DHTTYPE);
   Se usa simpleDHT22 porque la libreria DHT tiene un delay de 270ms
*/
SimpleDHT22 dht22(HUMPIN);
uint32_t delayMS;
int err = SimpleDHTErrSuccess;

String licorData;                 //Almacena la trama enviada por el licor.
String licorModel = "";           //Almacena modelo de licor. Se detecta en la primera trama leida por <li820> o <li850>
bool licorStringComplete = false; //Controla si se ha leido completamente la trama enviada por el licor para ser procesada.

//Tamaño de objeto json en memoria.
//Si se añaden sensores, nuevos campos calcular nuevo tamaño en : https://arduinojson.org/v6/assistant/
const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(11) + JSON_OBJECT_SIZE(12) + 310;
DynamicJsonDocument lastJsondata(capacity);
JsonObject lastLicor = lastJsondata.createNestedObject("licor");
JsonObject lastAna = lastJsondata.createNestedObject("ana");
//------------------------------------------------------------------------------
//Objeto Struct devuelto por la lectura de temperatura y humedad con libreria simpleDHT
struct TempHumSt
{
  float temp = 0.0;
  float hum = 0.0;
};
struct TempHumSt tempHum;

/* DHT 22 y 11 necesita un tiempo entre lecturas consecutivas de al menos 2 seg.
        Licor envia datos cada 0.5s
        control de lectura hace que se lea el sensor dht cada 10 tramas= 5 segundos
      ControlLectura
*/
int controlLectura = 0;
float dis = 0;
float caudal = 0;
//---------------------------------------CONTROL DE TRAMAS Y TIEMPOS---------------------------------------------------------
int nTramas = 0; // Contador de tramas enviadas por licor. Usado para imprimir cada 0.5 seg/1 segundos
float refTime;   // referencia de tiempo en tramas en ms
unsigned long nowTime = millis();
unsigned long beforeTime = millis();
//-----------------------------------------------------------------------------------------------------------------------------

//----------------------------------------SD---------------------------------------------------------
//SDCARD
#define SDCARD_CS_PIN 15 // PINES PARA TEENSY 3.2
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 14
//const int SD_CS = 15;                     //Para arduino
// set up variables using the SD utility library functions:
// Sd2Card card;
// SdVolume volume;
// SdFile root;
// Arduino mega: pin 4

//File ficheroDatos;
bool sdAvailable = false; // indica si se ha podido inicializar la sd y está disponible.
bool saveSD = false;      // guarda o no guardar en la sd. Si esta conectada, cambia a true en inicializaSD
String fechaPunto;        // Almacenará la fecha de GPS cuando se inicia la medida por bluetooth (recibe un 1)
//-----------------------------------------------------------------------------------------------
// Conversores AD
Adafruit_ADS1115 ads0(0x48); //addrees conectado a GND
Adafruit_ADS1115 ads1(0x49); //addrees conectado a VCC
const float multiplier = 0.1875F;
const float multiplier2 = 0.1875F;
// const float multiplier =0.125F;// ads0.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// const float multiplier =0.0625F; // ads0.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// const float multiplier =0.03125F;// ads0.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// const float multiplier =0.015625F;//ads0.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// const float multiplier =0.0078125F;//ads0.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

//----------------------------------------CONFIGURACIÓN DE RANGO DEL CONVERSOR AD PARA LECTURA DE LICOR---------------------------------------------------------
float co2_range = 5000.00; //Para convertir de de voltios a c02 según la configuración del licor.
float co2_min = 0.0;       //co2 = volt * ((co2_range - co2_min) / vol_range) + co2_min ;
float vol_range = 5.0;
float h2o_range = 1000.0; //Para convertir de de voltios a c02 según la configuración del licor.
float h2o_min = 0.0;      //co2 = volt * ((co2_range - co2_min) / vol_range) + co2_min ;
//----------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------CONFIGURACIÓN LCD---------------------------------------------------------
//#define BACKLIGHT_LED 10
#define SID_PIN 18
#define SCLK_PIN 19
#define RS_PIN 20
#define RESET_LCD_PIN 21
#define CS_PIN 22

ST7565 glcd(SID_PIN, SCLK_PIN, RS_PIN, RESET_LCD_PIN, CS_PIN);

//---------------------------------------FIN CONFIGURACIÓN LCD---------------------------------------------------------

//---------------------------------------------GPS SETUP---------------------------------------------------------
#if !defined(GPS_FIX_TIME) | !defined(GPS_FIX_DATE)
#error You must define GPS_FIX_TIME and DATE in GPSfix_cfg.h!
#endif

#if !defined(NMEAGPS_PARSE_RMC) & !defined(NMEAGPS_PARSE_ZDA)
#error You must define NMEAGPS_PARSE_RMC or ZDA in NMEAGPS_cfg.h!
#endif

#if !defined(GPS_FIX_LOCATION)
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SPEED)
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SATELLITES)
#error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif
#if !defined(GPS_FIX_TIME) | !defined(GPS_FIX_DATE)
#error You must define GPS_FIX_TIME and DATE in GPSfix_cfg.h!
#endif

#if !defined(NMEAGPS_PARSE_RMC) & !defined(NMEAGPS_PARSE_ZDA)
#error You must define NMEAGPS_PARSE_RMC or ZDA in NMEAGPS_cfg.h!
#endif

#if !defined(GPS_FIX_LOCATION)
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SPEED)
#error You must uncomment GPS_FIX_SPEED in GPSfix_cfg.h!
#endif

#if !defined(GPS_FIX_SATELLITES)
#error You must uncomment GPS_FIX_SATELLITES in GPSfix_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#define PC_BAUDRATE 115200L
#define GPS_DEFAULT_BAUDRATE 9600L  // Default baudrate is determined by the receiver manufacturer.
#define GPS_WANTED_BAUDRATE 115200L // Wanted buadrate at the moment can be 9600L (not changed after defaults) or 115200L (changed by the
// `changeBaudrate()` function with a prepared message).
// Array of possible baudrates that can be used by the receiver, sorted descending to prevent excess Serial flush/begin
// after restoring defaults. You can uncomment values that can be used by your receiver before the auto-configuration.
const long possibleBaudrates[] = {
    //921600L,
    //460800L,
    //230400L,
    115200L,
    //57600L,
    //38400L,
    //19200L,
    9600L,
    //4800L,
};

static NMEAGPS gps; // This parses the GPS characters
static gps_fix fix; // This contains all the parsed pieces
using namespace NeoGPS;

struct DateTimeStr
{
  String dtime = "00.00.00";
  String date = "00-00-2000";
  String datebig = "2000-00-00";
};
DateTimeStr datetimestr;
float flat, flon, falt;
boolean gpsFijado = false; // Controla cuando el gps está fijado para procesar tramas del licor

//---------------------------------------------GPS SETUP---------------------------------------------------------

//---------------------------------------------BOTONES-----------------------------------------------
GFButton inicioBoton(INICIO_B);
GFButton acumularBoton(ACUMULAR_B);
GFButton finBoton(FIN_B);
boolean inicio = false;   //Estado de los botones.
boolean acumular = false; //Secuencias posibles.  INICIO | INICIO | INICIO - > cerrar fichero y destruir
boolean fin = false;
//control mostrar lineas de display
boolean display34 = false;       //3-> t0----   c02  4-> media y stdev
boolean display56 = false;       // 5  ti  c02   6 ->  Flujo R2
boolean display7 = false;        // tfin   C02
boolean updateMediaStdev = true; //indica si valores de la linea 4 se actualizan-> media y stdev
boolean updateFlujoR2 = true;    // indica si valores de la linea 6 se actualizan-> Flujo y R2
boolean update7 = true;          //indica si valores de la linea 6 se actualizan-> tfin   C02

// ----------------------------------------FIN BOTONES -----------------------------------------------------------
// variables para guardar los calculos matematicos mostrados en display e impresos en csv
float t0 = 0.0;
float co2t0 = 0.0;
float media = 0.0;
float stdev = 0.0;
float ti = 0.0;
float co2ti = 0.0;
float flujo = 0.0;
float r2 = 0.0;
float tf = 0.0;
float co2tf = 0.0;
//----------------------------------------------CALCULOS MATEMATICOS------------------------------------------------
CircularBuffer<float, 50> bufferTiempo;
CircularBuffer<float, 50> bufferCO2;
//CircularBuffer<float> bufferTiempo(longMaxBuffer);            // Buffer circular almacena tiempo
//CircularBuffer<float> bufferCO2(longMaxBuffer);
//CircularBuffer<float> bufferH20(longMaxBuffer);
//CircularBuffer<float> bufferCO2ana(longMaxBuffer);
//CircularBuffer<float> bufferH20ana(longMaxBuffer);

// Struct para calculos en tiempo real
struct CalculoMath
{
  float mean = 0.0;
  float stdev = 0.0;
  float fluxPend = 0.0;
  float termIndep = 0.0;
  float r2 = 0.0;
};
CalculoMath calculosCO2;
//CalculoMath calculosH20;
//CalculoMath calculosCO2ana;
//CalculoMath calculosH20ana;
//----------------------------------------------FIN CALCULOS MATEMATICOS------------------------------------------------

//----------------------------------------------MÉTODOS Y FUNCIONES -----------------------------------------------
void inicializaDisplay();
void setupBotones();
void boton_inicio_press(GFButton &inicioBoton);
void boton_acumular_press(GFButton &acumularBoton);
void boton_fin_press(GFButton &finBoton);
static void setupGPS();
void restoreDefaults();
void changeBaudrate();
void sendPacket(byte *packet, byte len);
 void readGPS();
 void obtenerParametrosGps(const gps_fix &fix);
struct DateTimeStr obtenerFechaHora(NeoGPS::time_t dt);

void inicializaAds();
void inicializaSD();
void setupReles();
void testBomba(int bombaNum);
String xmlTakeParam(String inStr, String needParam);
void serialEvent3();
struct TempHumSt readSimpleTempHum();
 void setupUltrasonido();
float readUlt();
float microsecondsToCentimeters(long microseconds);
float getCanalAnal(int line, Adafruit_ADS1015 &ads);
float voltToCO2ppm(float mvolt);
float voltToH2Oppm(float mvolt);
float voltToSCCM(float mvolt);
float voltToBarpress(float mvolt);
void serialEvent2();
void nuevoFichero();
void generarFileName(DynamicJsonDocument &lastJsondata, char* dest);
String generateTramaData(DynamicJsonDocument &doc, bool writeTime );
String jsonTOcsvline(DynamicJsonDocument &doc );
String cabeceraCsv(DynamicJsonDocument doc);
void printbuffer();
void showDisplay(DynamicJsonDocument &doc);
void imprimirFechaHora(String fecha, boolean write);
void imprimirLatLon(float lat, float lon, boolean gpsFijado, boolean write);
void imprimirCelltempCellpressIvolt(DynamicJsonDocument &doc );
void imprimirTC02MediaStdev( );
void imprimirTC02FlujoR2();
void imprimirTC02Fin();
void imprimeCalculos();
struct CalculoMath calcularMath();


//----------------------------------------------SETUP------------------------------------------------
void setup()
{
  // inicializaDisplay();
  // setupBotones();
  // setupUltrasonido();

  USB_PORT.begin(115200);
  while (!USB_PORT)
  {
    ;
  }
  USB_PORT.println(F("Estación iniciando"));

  // setupGPS();

  BT_PORT.begin(115200); //Bluetooth
  while (!BT_PORT)
  {
    ;
  }
  USB_PORT.println(F("Bluetooth listo."));

  // inicializaAds();
  // inicializaSD();
  delay(500); // tiempo de estabilización

  // setupReles();

  //licorData.reserve(1000);                        // reserve 1000 bytes for the licorData:
  //delay(1000); // tiempo de estabilización

  bufferCO2.clear(); //Limpiamos el buffer -> los calculos durante los estados son independientes
  //
  

  // glcd.clear();
  // inicializamos licor al final.
  LICOR_PORT.begin(9600); //Licor sensor
  while (!LICOR_PORT)
  {
    ;
  }
  LICOR_PORT.flush();
  USB_PORT.println(F("Licor sensor ready. "));
  // noInterrupts();
  // while (!gpsPort.available())
  // {
  //     glcd.drawstring(0, 0, "esperando gps ");
  //     glcd.display();
  //     USB_PORT.println(F("esperando gps "));

  // }
  // readGPS();
  // interrupts();
  
  refTime = 0;          //RefTime variable contadora de tiempo mientras muestrea
  nowTime = millis();   // nº ms desde que arduino arranca
  beforeTime = nowTime; // refTime = refTime + (nowTime - beforeTime) antes y despues de la lectura de trama
  
}
//-----------------------------------------------------------LOOP----------------------------------------------
void loop()
{
  //  if (gpsPort.available())
  //    readGPS();

  // inicioBoton.process();
  // acumularBoton.process();
  // finBoton.process();

  //if (licorStringComplete)
  //{
  if (millis()- beforeTime>=1000){
    nowTime = millis(); //refTime se resetea cuando se recibe un 0 o un 1 por bluetooth (apagado/encendido bomba)
    refTime = refTime + (nowTime - beforeTime) + float(random(980,1020))/1000;
    beforeTime = nowTime;
    //Activamos las mediciones
    digitalWrite(LEDPIN, HIGH);          // led onn
    unsigned long inicioProc = millis(); //variable que controla/mide el tiempo de proceso de informacion
    controlLectura += 1;
    //Crea el objeto JSON
    DynamicJsonDocument doc(capacity);
    JsonObject licor = doc.createNestedObject("licor");
    JsonObject ana = doc.createNestedObject("ana");

    /* DHT 22 y 11 necesita un tiempo entre lecturas consecutivas de al menos 2 seg.
        Licor envia datos cada 0.5s
        control de lectura hace que se lea el sensor dht cada 10 tramas= 5 segundos
      ControlLectura
    */
    // if ((controlLectura % 10) == 0 || tempHum.temp == -10000)
    // {

      //Serial.println(licorData.substring(0, licorData.length() - 1));
      tempHum = readSimpleTempHum();
      //if (tempHum.temp==-10000 && tempHum.hum==-10000 ){                  // en caso de error de lectura del sensor envio ultimo valor
      // tempAmb = tempHum.temp;
      // humAmb = tempHum.hum;
      //}
      //Serial.println(millis());
    //}
    controlLectura += 1;
    dis = readUlt();
    // CAMBIOS PARA TEENSY 3.2
    //caudal = readAnalog(CAUDALIMETRO_PIN);
    caudal = voltToSCCM(getCanalAnal(CAUDALIMETRO, ads0));
    //Serial.println(licorData.substring(1, licorData.length() - 1));
    
    doc["time"] = refTime / 1000.0; //pasamos a segundos
    //bufferTiempo.push(doc["time"]); // añadimos al buffer
    doc["tempAmb"] = tempHum.temp;
    doc["humAmb"] = tempHum.hum;
    doc["dis"] = dis;
    doc["caudal"] = caudal;
    //doc["lat"] = flat;
    doc["lat"] = 28.46894;
    //doc["lon"] = flon;
    doc["lon"] = -16.29558;
    // doc["alt"] = falt;
    doc["alt"] = 357.7;
    doc["date"] = "2019-11-23";
    // doc["date"] = datetimestr.datebig;
    doc["gpsTime"] = "08.11.26";
    // doc["gpsTime"] = datetimestr.dtime;
    licorModel="li850";
    licor["model"] = licorModel;
    
    // licor["celltemp"] = xmlTakeParam(licorData, "celltemp");
    // licor["cellpres"] = xmlTakeParam(licorData, "cellpres");
    // licor["co2"] = xmlTakeParam(licorData, "co2");
    licor["celltemp"] = sci(float(random(50000,52000))/1000,6);
    licor["cellpres"] = sci(float(random(50000,52000))/1000,6);
    licor["co2"] = sci(50*(refTime/1000) + 400 + (random(10,90)),6);   // generamos una recta
    //String licor_co2 = licor["co2"];
    //bufferCO2.push(licor_co2.toFloat());

    //licor["co2abs"] = xmlTakeParam(licorData, "co2abs");
    licor["co2abs"] = sci(float(random(900000,955000))/1000,6);

    if (licorModel.startsWith("li850"))
    { //Solo para Licor 850
      // licor["h2o"] = xmlTakeParam(licorData, "h2o");
      licor["h2o"] = sci(float(random(17000,18000))/1000,6);
      //String licor_h2o = licor["co2"];
      //bufferH20.push(licor_h2o.toFloat());
      // licor["h2oabs"] = xmlTakeParam(licorData, "h2oabs");
      licor["h2oabs"] = sci(float(random(9000,1000))/1000,6);
      // licor["h2odewpoint"] = xmlTakeParam(licorData, "h2odewpoint");
      licor["h2odewpoint"] = sci(float(random(15000,16000))/1000,6);
      licor["h2oana"] = voltToH2Oppm(getCanalAnal(LICOR_H2O_ANA, ads0)); // lectura canal analogico Conversor AD. Suponemos h2o en la cofiguracion licor -> PIN 7 licor 850
      //bufferH20ana.push(licor["vout1"]);
    }
    // licor["ivolt"] = xmlTakeParam(licorData, "ivolt");
    // licor["flowrate"] = xmlTakeParam(licorData, "flowrate");
    licor["ivolt"] = sci(float(random(11900,12500))/1000,6);
    licor["flowrate"] = float(random(998000,1100000))/1000;
    licor["co2ana"] = voltToCO2ppm(getCanalAnal(LICOR_CO2_ANA, ads0)); // lectura canal analogico Conversor AD. Suponemos C02  en la cofiguracion licor -> PIN 7 licor 850
    //bufferCO2ana.push(licor["vout1"]);
    ana["sccm"] = caudal;
    //ana["bar_press"] = voltToBarpress(getCanalAnal(PRES_BAR, ads0));

    ana["A10"] = getCanalAnal(A10,ads1);
    ana["A11"] = getCanalAnal(A11,ads1);
    ana["A12"] = getCanalAnal(A12,ads1);
    ana["A13"] = getCanalAnal(A13,ads1);

    serializeJson(doc, USB_PORT); USB_PORT.println(); //Envia json por puerto serie (USB)
    serializeJson(doc, BT_PORT);  Serial.println();       //Envia json por bluetooth
    //String trama = generateTramaData( doc, true );          //Envia trama por puerto bluetooth (true imprime TIME, false No
    //USB_PORT.println(trama);                                //Envia trama por puerto serie (USB)
    //BT_PORT.println(trama);

    //printbuffer();

    // calculosCO2 = calcularMath();
    // CalculoMath calculosH20;
    // CalculoMath calculosCO2ana;
    // CalculoMath calculosH20ana;

    // showDisplay(doc);

    // if (saveSD) {
    //    ficheroDatos.println( jsonTOcsvline(doc) );         //Guarda linea csv con todos los parámetros en el fichero
    //    ficheroDatos.flush();
    // }

    lastJsondata = doc;
    lastLicor = licor;
    lastAna = ana;

    licorData = "";
    licorStringComplete = false;
    digitalWrite(LEDPIN, LOW);
    unsigned long finProc = millis() - inicioProc;
    inicioProc = finProc;

    //Serial.println(finProc);                            // Imprime tiempo que tarda el proceso
  }

  
}

//-------------------------------------------------------- FIN LOOP----------------------------------------------

//--------------------------------------------------------DISPLAY---------------------------------------------

/*
Inicializar display LCD
*/
void inicializaDisplay()
{
  // pinMode(BACKLIGHT_LED, OUTPUT);
  // digitalWrite(BACKLIGHT_LED, HIGH);

  glcd.st7565_init();
  glcd.st7565_command(CMD_DISPLAY_ON);
  glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
  glcd.st7565_set_brightness(0x18);
  glcd.display(); // show splashscreen
  delay(1000);
  glcd.clear();
}

/*
   //Mostrar en display segun condiciones.

*/
void showDisplay(DynamicJsonDocument &doc) {
  glcd.clear();
  imprimirFechaHora(datetimestr.date + " " + datetimestr.dtime, true);

  if ( fix.valid.time && fix.valid.date && fix.valid.location && fix.valid.satellites )
    imprimirLatLon(flat, flon, true, true);  // lat, long, gpsfijado, write
  else
    imprimirLatLon(flat, flon, false, true);  // lat, long, gpsfijado, write

  imprimirCelltempCellpressIvolt(doc);

  if (display34 ) {
    imprimirTC02MediaStdev( );
  }
  if (display56 ) {
    imprimirTC02FlujoR2( ) ;
  }
  if (display7) {
    imprimirTC02Fin( );
    //display7=false;
  }

  glcd.display();
}


/*
   Imprime fecha y hora en linea 0.
*/
void imprimirFechaHora(String fecha, boolean write) {
  if (write) {
    int MAX = fecha.length() + 1;
    char fecha_char [MAX];
    fecha.toCharArray(fecha_char, MAX);
    glcd.drawstring(0, 0, fecha_char);
    //glcd.display();
  }

}
/*
   imprime en linea 1 (comienza en 0) del lcd latitud, longitud y si el gps está fijado o no.
   write indica si imprime o no.
*/
void imprimirLatLon(float lat, float lon, boolean gpsFijado, boolean write) {
  if (write) {
    char caritaSonriente = 002;
    char admiracion = 19;

    String latitud = String(lat, 4);
    String longitud = String(lon, 4);
    int longLine = latitud.length() + longitud.length() + 7; // (+28.1225,-16.8549) 1
    String lineStr;
    char linechar[longLine];

    lineStr = "(" + latitud + "," + longitud + ")";

    lineStr.toCharArray(linechar, longLine);

    glcd.drawstring(0, 1, linechar);

    if (gpsFijado) {
      glcd.drawchar(120, 1, caritaSonriente);    // imprime carita sonriente si el gps está fijado.
    } else {
      glcd.drawchar(120, 1, admiracion);    // imprime admiracion si el gps NO está fijado.
    }
    //glcd.display();
  }
}

/*
   Imprime en linea 2 cellTemp:51.4, P: 0.92, V: 12.1
*/
void imprimirCelltempCellpressIvolt(DynamicJsonDocument &doc ) {
  JsonObject licor = doc["licor"];
  String licor_celltemp = licor["celltemp"]; // "5.14571e1"
  String licor_cellpres = licor["cellpres"]; // "9.78824e1"
  String licor_ivolt = licor["ivolt"]; // "1.1445319e1"

  //Imprime en linea 2 T:28.0, P: 0.92, V: 12.1
  String line2 = "T:" + String(licor_celltemp.toFloat(), 1) + " P:" + String(licor_cellpres.toFloat(), 1) + " V:" + String(licor_ivolt.toFloat(), 2);
  int longLine2 = line2.length();
  char linechar2[longLine2];
  line2.toCharArray(linechar2, longLine2);
  glcd.drawstring(0, 2, linechar2);
}

/*
   Imprime en linea 3 y 4

*/
void imprimirTC02MediaStdev( ) {
//  Serial.print(t0); Serial.print('|'); Serial.println(co2t0);

  String line3 = "t0:" + String(t0, 2) + "  CO2:" + String(co2t0, 2);
  int longLine3 = line3.length();
  String line4 = "M:" + String(media, 2) + "  ST:" + String(stdev, 2);
  int longLine4 = line4.length();
  char linechar3[longLine3];
  char linechar4[longLine4];
//  Serial.println(line3 + "\n" + line4);
  line3.toCharArray(linechar3, longLine3);
  line4.toCharArray(linechar4, longLine4);
  glcd.drawstring(0, 3, linechar3);
  glcd.drawstring(0, 4, linechar4);

}

/*
   Imprime en linea 5 y 6

*/
void imprimirTC02FlujoR2() {
//  Serial.print(ti); Serial.print('|'); Serial.println(co2ti);
  String line5 = "ti:" + String(ti, 2) + " CO2:" + String(co2ti, 2);
  int longLine5 = line5.length();
  String line6 = "F:" + String(flujo, 2) + " R2:" + String(r2, 2);
  int longLine6 = line6.length();
  char linechar5[longLine5];
  char linechar6[longLine6];
  line5.toCharArray(linechar5, longLine5);
  line6.toCharArray(linechar6, longLine6);
  glcd.drawstring(0, 5, linechar5);
  glcd.drawstring(0, 6, linechar6);

}

/*
   Imprime en linea 7
*/
void imprimirTC02Fin() {
//  Serial.print(tf); Serial.print('|'); Serial.println(co2tf);
  String line = "tf:" + String(tf, 2) + " CO2:" + String(co2tf, 2);
  int longLine = line.length();
  char linechar[longLine];
  line.toCharArray(linechar, longLine);
//  Serial.println(linechar);
  glcd.drawstring(0, 7, linechar);
}
//-----------------------------------------------------------FIN DISPLAY------------------------------------------------------------


//-----------------------------------------------------------BOTONES------------------------------------------------------------
/*
   Configuración resistencias pullup para los botones inicio,acumular, fin
*/
// void setupBotones()
// {
//   // se han puesto resistencias. Descartamos internas.
//   //  pinMode(INICIO_B, INPUT_PULLUP);
//   //  pinMode(ACUMULAR_B, INPUT_PULLUP);
//   //  pinMode(FIN_B, INPUT_PULLUP);

//   pinMode(INICIO_B, INPUT);
//   pinMode(ACUMULAR_B, INPUT);
//   pinMode(FIN_B, INPUT);

//   // CONFIGURAR MANEJADORES DE EVENTOS (FUNCIONES CALLBACK)
//   inicioBoton.setPressHandler(boton_inicio_press);
//   acumularBoton.setPressHandler(boton_acumular_press);
//   finBoton.setPressHandler(boton_fin_press);

//   // CONFIGURAR TEMPORIZADOR DE REBOTES EN 100 MILISEGUNDOS
//   //inicioBoton.setDebounceTime(5);
//   //acumularBoton.setDebounceTime(5);
//   //finBoton.setDebounceTime(5);
// }

/*
   NOTA SOBRE SECUENCIA DE BOTONES:  (ESTADO DE INICIO= ESTADO DE ESTABILIZACIÓN (CALCULO DE MEDIA ARITMETICA CON CAMPANA EN EL AIRE)
   Secuencia correcta:   INICIO ACUMULAR FIN   -----> SOLO CON LA SECUENCIA CORRECTA SE GUARDA EL FICHERO
                                               -----> CIERRA EL FICHERO Y CREA UN TEMPORAL PARA ASEGURAR
                                               -----> QUE EN PROXIMO INICIO NO SE DESTRUYA ESE FICHERO. ES DECIR, SOLO SE GUARDARA SI SE PULSA FIN.
   Secuencias posibles:  INICIO, INICIO INICIO.....  --> EL FICHERO SE DESTRUYE Y CREA UNO NUEVO CADA VEZ QUE PULSE (NO SE GUARDA)
                         INICIO, ACUMULAR, INICIO    --> EL FICHERO SE DESTRUYE Y SE CREA UNO NUEVO AL VOLVER A PULSAR INICIO
                         INICIO, FIN  ( NO FUNCIONA) --> NO HACE NADA -> EL FICHERO SIGUE GUARDANDO
                         SOLO SE COMIENZA POR INICIO


*/

/*
    MANEJADOR DE EVENTOS DEL BOTON DE INICIO
*/
// void boton_inicio_press(GFButton &inicioBoton)
// {
//   //if (inicio) {
//   Serial.println(F("Aqui se hacen las acciones de medida inicial"));
//   digitalWrite(RELE_BOMBA, LOW);  // activos a nivel bajo siempre
//   refTime = 0;
//   ficheroDatos.close(); // Por si recibe 2 unos seguidos. Cerramos ficheros.
//   if (SD.exists(ficheroDatos.name()))
//   {                                 // destruimos el fichero siempre que exista.
//     SD.remove(ficheroDatos.name()); //Para secuencias incorrectas
//   }
//   // debe Toma fecha de GPS
//   saveSD = true; // habilitamos guardado del csv en el loop principal
//   if (sdAvailable) {
//      nuevoFichero();                               // genera nuevo fichero igual que con el bluetooth
//   }

//   t0 = 0.0;
//   co2t0 = bufferCO2.last();
//   bufferTiempo.clear();
//   bufferCO2.clear(); //Limpiamos el buffer -> los calculos durante los estados son independientes
//   //
//   inicio = true;    //Reset varibles que controlan los botones
//   acumular = false; //Pulsado inicio solo se puede pulsar acumular
//   fin = false;

//   display34 = true;  //mostramos solo las líneas 34   (t0 co2 y media y stdev)
//   display56 = false; //no se muestran
//   display7 = false;  //no se muestran

//   updateMediaStdev = true; //indica que la linea 4 de media y stdev se van a actualizar conforme llegan datos del licor
//   updateFlujoR2 = true;    //lo mismo para linea 6 de flujo y stdev (esta no se muestra hasta pulsar boton acumular.
//   // }
// }

/*
    MANEJADOR DE EVENTOS DEL BOTON DE ACUMULAR
*/
// void boton_acumular_press(GFButton &acumularBoton)
// {
//   USB_PORT.println(F("Aqui se hacen las acciones de acumular: "));
//   if (inicio && !acumular)
//   {

//     ti = bufferTiempo.last();
//     co2ti = bufferCO2.last();

//     bufferCO2.clear(); //Limpiamos el buffer -> los calculos durante los estados son independientes
//     //
//     acumular = true; //Reset varibles que controlan los botones
//     fin = false;     //Pulsado acumular solo se puede pulsar fin

//     //display34=false;                             //se deja de actualizar las lineas 3 y 4
//     display56 = true; //Se muestra linea 5 6 con ti c02 y  flujo y R2
//     //display7=false;                              //no se muestran
//     updateMediaStdev = false; //deja de actualizar linea 4 display  media y stdev
//   }
// }
/*
    MANEJADOR DE EVENTOS DEL BOTON DE FIN
*/
// void boton_fin_press(GFButton &finBoton)
// {
//   USB_PORT.println(F("Aqui se hacen las acciones de fin de medida: "));
//   if (inicio && acumular && !fin)
//   {

//     digitalWrite(RELE_BOMBA, HIGH);
//     USB_PORT.println(F("Turn off LED"));
//     refTime = 0;
//     saveSD = false;

//     tf = bufferTiempo.last(); //Previo a imprimir calculos guarda en las variables los ultimos datos del buffer
//     co2tf = bufferCO2.last();
//     if (sdAvailable)
//     {
//       imprimeCalculos();                               // genera nuevo fichero igual que con el bluetooth
//     }
//     ficheroDatos.close();
//     ficheroDatos = SD.open("temp.temp", FILE_WRITE); // se crea un fichero temporal para controlar y asegurar que
//     USB_PORT.println(F("Fichero Cerrado:"));            // al crear un nuevo fichero se cierre y destruya si existe.
//     //bufferTiempo.clear();
//     bufferCO2.clear(); //Limpiamos el buffer -> los calculos durante los estados son independientes
//     inicio = false;    //Reset varibles que controlan los botones.
//     fin = false;       //Pulsado fin solo se puede pulsar inicio

//     //display34=false;                                 //se deja de actualizar las lineas 3 y 4
//     //display56=false;                                //se deja de actualizar las lineas 5 y 6
//     display7 = true;       //Se muestra linea 7 con ti c02
//     updateFlujoR2 = false; //deja de actualizar flujo y R2
//   }
// }
//--------------------------------------------------------FIN BOTONES------------------------------------------------------------

//-----------------------------------------------------SETUP GPS --------------------------------------------
// static void setupGPS()
// {
//   USB_PORT.println(F("Buscando GPS en puerto " GPS_PORT_NAME));
//   // Restore the receiver default configuration.
//   for (byte i = 0; i < sizeof(possibleBaudrates) / sizeof(*possibleBaudrates); i++)
//   {
//     USB_PORT.print("Trying to restore defaults at ");
//     USB_PORT.print(possibleBaudrates[i]);
//     USB_PORT.println(" baudrate...");

//     if (i != 0)
//     {
//       delay(100); // Little delay before flushing.
//       gpsPort.flush();
//     }

//     gpsPort.begin(possibleBaudrates[i]);
//     restoreDefaults();
//   }
//   // Switch the receiver serial to the default baudrate.
//   if (possibleBaudrates[sizeof(possibleBaudrates) / sizeof(*possibleBaudrates) - 1] != GPS_DEFAULT_BAUDRATE)
//   {
//     USB_PORT.print(F("Switching to the default baudrate which is "));
//     USB_PORT.print(GPS_DEFAULT_BAUDRATE);
//     USB_PORT.println("...");

//     delay(100); // Little delay before flushing.
//     gpsPort.flush();
//     gpsPort.begin(GPS_DEFAULT_BAUDRATE);
//   }

//   // Switch the receiver serial to the wanted baudrate.
//   if (GPS_WANTED_BAUDRATE != GPS_DEFAULT_BAUDRATE)
//   {
//     USB_PORT.print(F("Switching receiver to the wanted baudrate which is "));
//     USB_PORT.print(GPS_WANTED_BAUDRATE);
//     USB_PORT.println("...");

//     changeBaudrate();

//     delay(100); // Little delay before flushing.
//     gpsPort.flush();
//     gpsPort.begin(GPS_WANTED_BAUDRATE);
//   }

//   USB_PORT.println(F("Auto-configuration is complete!"));

//   delay(100); // Little delay before flushing.
//   gpsPort.flush();
//   USB_PORT.println(F("GPS listo."));
// }

// // Send a packet to the receiver to restore default configuration.
// void restoreDefaults()
// {
//   // CFG-CFG packet.
//   byte packet[] = {
//       0xB5, // sync char 1
//       0x62, // sync char 2
//       0x06, // class
//       0x09, // id
//       0x0D, // length
//       0x00, // length
//       0xFF, // payload
//       0xFF, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0xFF, // payload
//       0xFF, // payload
//       0x00, // payload
//       0x00, // payload
//       0x17, // payload
//       0x2F, // CK_A
//       0xAE, // CK_B
//   };

//   sendPacket(packet, sizeof(packet));
// }

// // Send a packet to the receiver to change baudrate to 115200.
// void changeBaudrate()
// {
//   // CFG-PRT packet.
//   byte packet[] = {
//       0xB5, // sync char 1
//       0x62, // sync char 2
//       0x06, // class
//       0x00, // id
//       0x14, // length
//       0x00, // length
//       0x01, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0xD0, // payload
//       0x08, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0xC2, // payload
//       0x01, // payload
//       0x00, // payload
//       0x07, // payload
//       0x00, // payload
//       0x03, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0x00, // payload
//       0xC0, // CK_A
//       0x7E, // CK_B
//   };

//   sendPacket(packet, sizeof(packet));
// }

// // Send the packet specified to the receiver.
// void sendPacket(byte *packet, byte len)
// {
//   for (byte i = 0; i < len; i++)
//   {
//     gpsPort.write(packet[i]);
//   }

//   //printPacket(packet, len);
// }

//--------------------------------------------------FIN SETUP GPS --------------------------------------------
//--------------------------------------------------LECTURA GPS --------------------------------------------
/*
   Lectura del GPS de forma intensiva, es decir,
   siempre esta intentando leer, antes de la interrupición y
   justo despues de la interrupcion.
   Cuando la variable global gps puede codificar la trama esta
   puede retornar los valores.
*/

//void serialEvent1()
// void readGPS()
// {

//   //while (gps.available(gpsPort))
//   if (gps.available(gpsPort))
//   {
//     fix = gps.read();
//     obtenerParametrosGps(fix);
//   }
// }

// void obtenerParametrosGps(const gps_fix &fix)
// {
//   // if ( fix.valid.time && fix.valid.date && fix.valid.location && fix.valid.satellites ) {
//   if (fix.valid.time && fix.valid.date)
//   {
//     datetimestr = obtenerFechaHora(fix.dateTime);
//     flat = fix.latitude();
//     flon = fix.longitude();
//     falt = fix.altitude();

//     gpsFijado = true;
//     String date=datetimestr.date;
//     String time=datetimestr.dtime;
//     char copy[21];
//     time.toCharArray(copy,21);
    
//     USB_PORT.println(datetimestr.date + " " + datetimestr.datebig + " " + datetimestr.dtime);
//     glcd.drawstring(0, 0, copy);
//     glcd.display();
//     USB_PORT.print(flat, 5);
//     USB_PORT.print(',');
//     USB_PORT.print(flon, 5);
//     USB_PORT.print(',');
//     USB_PORT.print(falt, 2);
//     USB_PORT.println();
//   }
// }

/*
   Retorna un objeto con date (1 con formato bigendia y otro con little endian)  y la hora.
   formato big endian (aaaa-mm-dd hh.min)
*/
struct DateTimeStr obtenerFechaHora(NeoGPS::time_t dt)
{
  DateTimeStr datetimestr;

  char dtime[] = "00.00.00";
  char date[] = "00-00-2000";
  char datebig[] = "2000-00-10"; //

  // Esta fumada reempla los datos en los arrays
  dtime[6] = dt.seconds / 10 + '0';
  dtime[7] = dt.seconds % 10 + '0';
  dtime[3] = dt.minutes / 10 + '0';
  dtime[4] = dt.minutes % 10 + '0';
  dtime[0] = dt.hours / 10 + '0';
  dtime[1] = dt.hours % 10 + '0';

  date[8] = (dt.year / 10) % 10 + '0';
  date[9] = (dt.year % 10) + '0';
  date[3] = dt.month / 10 + '0';
  date[4] = dt.month % 10 + '0';
  date[0] = dt.date / 10 + '0';
  date[1] = dt.date % 10 + '0';
  datetimestr.date = String(date);

  datebig[8] = dt.date / 10 + '0';
  datebig[9] = dt.date % 10 + '0';
  datebig[5] = dt.month / 10 + '0';
  datebig[6] = dt.month % 10 + '0';
  datebig[2] = (dt.year / 10) % 10 + '0';
  datebig[3] = dt.year % 10 + '0';
  datetimestr.datebig = String(datebig);

  datetimestr.dtime = String(dtime);

  return datetimestr;
}

/*Convierte a timestamp*/
unsigned long unixTimestamp(int year, int month, int day,
              int hour, int min, int sec)
{
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
 
  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;
 
  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;
 
  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
 
  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}

//-------------------------------------------------------------FIN LECTURA GPS---------------------------------------------------------

//---------------------------------------------------------CONVERSORES AD  ADS1115----------------------------------------------------------
/*
  Inicializa conversor ad
*/
// void inicializaAds()
// {
//   // Descomentar el que interese
//   Wire.setSCL(SCL0_PIN); // Fija pines sda y scl0 a 16 y 17
//   Wire.setSDA(SDA0_PIN);
//   ads0.setGain(GAIN_TWOTHIRDS); //+/- 6.144V  1 bit = 0.1875mV (default)
//   ads1.setGain(GAIN_TWOTHIRDS); //+/- 6.144V  1 bit = 0.1875mV (default)
//   // ads.setGain(GAIN_ONE);     //   +/- 4.096V  1 bit = 0.125mV
//   // ads.setGain(GAIN_TWO);     //   +/- 2.048V  1 bit = 0.0625mV
//   // ads.setGain(GAIN_FOUR);    //   +/- 1.024V  1 bit = 0.03125mV
//   // ads.setGain(GAIN_EIGHT);   //   +/- 0.512V  1 bit = 0.015625mV
//   // ads.setGain(GAIN_SIXTEEN); //   +/- 0.256V  1 bit = 0.0078125mV
//   ads0.begin();
//   ads1.begin();
//   USB_PORT.println(F("Conversores AD inicializados"));
// }

/*
Lectura del canal analógico del conversor ad
*/
float getCanalAnal(int line, Adafruit_ADS1015 &ads)
{
  float mvolt;
  int16_t adc;

  // adc = ads.readADC_SingleEnded(line);
  // mvolt = adc * multiplier;
  mvolt = random(3000000,4100000)/1000;

  return mvolt;
}

float voltToCO2ppm(float mvolt)
{
  float co2;
  co2 = (mvolt / 1000.0) * ((co2_range - co2_min) / vol_range) + co2_min;
  return co2;
}

float voltToH2Oppm(float mvolt)
{
  float h2o;
  h2o = (mvolt / 1000.0) * ((h2o_range - h2o_min) / vol_range) + h2o_min;
  return h2o;
}
/*
Transforma milivoltios a sscm (Unidades de flujo)  cm³/min. 
*/
float voltToSCCM(float mvolt)
{
  //https://sensing.honeywell.com/honeywell-sensing-airflow-awm3000-series-catalog-pages.pdf
  // intepolación polinomica del caudalimetros y = 7E-05x2 - 0.2129x + 164.16
  float sccm = 0.00007 * (mvolt * mvolt) - 0.2129 * mvolt + 164.16;
  return sccm;
}
/*
voltios a presion barométrica para sensor analógico VAISALA 1100 - 500
Calculo del multiplicador:
m= (1100-500)/(5000-0)= 600/5000=0.12   -->  0.24 para sensores 2.5 vol max
donde 1100= max bar
500 =min press
5000 y 0 maximo y min voltaje del sensor.
*/
float voltToBarpress(float mvolt)
{
  //y = 0.12 𝑚𝑏𝑎𝑟/𝑚𝑉 + 500 mbar
  float barpress;
  barpress = 0.12 * mvolt + 500;
  return barpress;
}
//---------------------------------------------------------FIN CONVERSORES AD----------------------------------------------------------
//------------------------------------------------------INICIALIZA SD--------------------------------
/*
   Inicializar sd
*/
// void inicializaSD()
// {
//   //uint32_t volumesize;
//   Serial.print("Inicializando SD card...");
//   SPI.setMOSI(SDCARD_MOSI_PIN);
//   SPI.setSCK(SDCARD_SCK_PIN);
//   if (!SD.begin(SDCARD_CS_PIN))
//   {
//     USB_PORT.println(F("Inicialización ha fallado!"));
//     sdAvailable = false;
//     saveSD = false;
//   }
//   else
//   {
//     USB_PORT.println(F("Inicialización Correcta."));
//     //saveSD=true;                                  // solo se hace saveSD=true cuando recibimos por bluetooth un 1.
//     sdAvailable = true;
//   }
//   // we'll use the initialization code from the utility libraries
//   // since we're just testing if the card is working!
//   if (!card.init(SPI_HALF_SPEED, SDCARD_CS_PIN))
//   {
//     USB_PORT.println("SD no inicializada:");
//     sdAvailable = false;
//     saveSD = false;
//   }
//   else
//   {
//     USB_PORT.println("SD Iniciado correctamente.");
//     sdAvailable = true;
//   }

//   // No Funciona para tarjetas > 2GB y slot velleman VMA304
//   //  if (!volume.init(card)) {
//   //    Serial.println("F(Formato de tarjeta No compatible. Usar FAT16/FAT32."));
//   //    while (1);
//   //  }
//   //  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
//   //  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
//   //  volumesize /= 2;                          // SD card blocks are always 512 bytes (2 blocks are 1KB)
//   //  volumesize /= 1024;
//   //  Serial.println("F(Tamaño de tarjeta (Mb):  "+ String(volumesize,2));
// }

void setupReles()
{
  pinMode(LEDPIN, OUTPUT);
  pinMode(RELE_BOMBA, OUTPUT);
  digitalWrite(RELE_BOMBA, HIGH);
  pinMode(BOMBA_INYECCION, OUTPUT);
  digitalWrite(BOMBA_INYECCION, HIGH);
  USB_PORT.println(F("Bombas activadas"));
  testBomba(RELE_BOMBA);
  testBomba(BOMBA_INYECCION);
}

/*
test de bombas al arrancar
*/
void testBomba(int bombaNum)
{
  delay(TIEMPO_TEST_B);
  digitalWrite(bombaNum, LOW); //Bomba desactivada a nivel alto.
  delay(TIEMPO_TEST_B);
  digitalWrite(bombaNum, HIGH); //Bomba desactivada a nivel alto.
}

//-------------------------------------------------------------------LICOR-------------------------------------------
/*
   Extrae needParam de la trama del licor
*/
String xmlTakeParam(String inStr, String needParam)
{
  if (inStr.indexOf("<" + needParam + ">") > 0 && inStr.indexOf("</" + needParam + ">") > 0)
  {
    int CountChar = needParam.length();
    int indexStart = inStr.indexOf("<" + needParam + ">");
    int indexStop = inStr.indexOf("</" + needParam + ">");
    return inStr.substring(indexStart + CountChar + 2, indexStop);
  }
  return "NF";
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial 3 RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.

  Lee puerto serie al ritmo del LICOR
*/
void serialEvent3()
{

  while (LICOR_PORT.available() > 0)
  {
    // get the new byte:
    char inChar = (char)Serial3.read();
    // add it to the licorData:
    licorData += inChar;
    if (licorData.length() > 9)
    {
      String finalData = licorData.substring(licorData.length() - 9, licorData.length() - 1);

      if (licorData.endsWith("\n") && licorData.length() > 320)
      {

        //if (nTramas%2==1){ // Descomentar si queremos 1 trama por segundo
        licorStringComplete = true;
        //Serial.println(nTramas);
        //}
        //Cuenta nº de tramas enviadas por el licor.
        nTramas += 1;
        if (licorModel == "")
        {
          licorModel = finalData.substring(finalData.length() - 7, finalData.length() - 2);
          Serial.print(F("Licor model:"));
          Serial.println(licorModel);
        }
      }
    }
  }
}

//---------------------------------------------------------------FIN LICOR-------------------------------------------

//---------------------------------------------------------------DHT 22-------------------------------------------
/*
  Retorna un objeto  struct con la temperatura y humedad
  struct TempHumSt{
    float temp=0;
    float hum=0;
  }
*/
struct TempHumSt readSimpleTempHum()
{
  struct TempHumSt tempHum;
  float temperature = 0;
  float humidity = 0;
  // if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess)
  // {
  //   tempHum.temp = -10000;
  //   tempHum.hum = -10000;
  //   return tempHum;
  // }
  // tempHum.temp = temperature;
  // tempHum.hum = humidity;
  tempHum.temp = float(random(24000,26000))/1000;
  tempHum.hum = float(random(54000,66000))/1000;
  return tempHum;
}

//-----------------------------------------------------------FIN DHT 22-------------------------------------------
//--------------------------------------------------SETUP ULTRASONIDO---------------------------------------------------------------
void setupUltrasonido()
{
  pinMode(ULTPIN_TRIG, OUTPUT); //For ultrasonic sensor
  pinMode(ULTPIN_ECHO, INPUT);
}
//--------------------------------------------------FIN SETUP ULTRASONIDO---------------------------------------------------------------

//-----------------------------------------------------------ULTRASONIDO HC -SR04 -------------------------------------------
/*
  Lee el sensor de ultrasonido. Retorna distanca en centimetros.
*/
float readUlt()
{
  float distUltData;
  long timeUltData;
  //Clean pins ultrasonic
  // digitalWrite(ULTPIN_TRIG, LOW);
  // delayMicroseconds(5);
  // digitalWrite(ULTPIN_TRIG, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(ULTPIN_TRIG, LOW);

  // //Read the returned wave
  // pinMode(ULTPIN_ECHO, INPUT);
  // timeUltData = pulseIn(ULTPIN_ECHO, HIGH);
  // distUltData = microsecondsToCentimeters(timeUltData);
  //return distUltData;
  return float(random(10000,12000))/1000;
}

float microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return (float)microseconds / 29.0 / 2.0;
}

//-----------------------------------------------------------FIN ULTRASONIDO HC -SR04 -------------------------------------------
//------------------------------------------------------------BLUETOOTH EVENT------------------------------------------------------
/*
   lectura del bluetooth mediante eventos en el puerto serie.
*/
void serialEvent2() {
  while (Serial2.available() > 0) {
    char inByte = Serial2.read();
    switch (inByte) {
      case '0':
        digitalWrite(RELE_BOMBA, HIGH); //activa a nivel bajo
        USB_PORT.println(F("Turn off LED"));
        refTime = 0;
        saveSD = false;
        //ficheroDatos.close();
        //USB_PORT.println(F("Fichero Cerrado:"));
        break;
      case '1':
        digitalWrite(RELE_BOMBA, LOW);
        USB_PORT.println(F("Turn on LED"));
        refTime = 0;
        saveSD = true;
        //ficheroDatos.close();                                             // Por si recibe 2 unos seguidos. Cerramos ficheros.
        // debe Toma fecha de GPS
        if (sdAvailable) {
          //nuevoFichero();
        }
        break;
    }
  }
}
//-----------------------------------------------------------------FICHEROS-------------------------------------------------------------
/*
   genera nuevo fichero con su nombre, datos iniciales y cabecera
*/
// void nuevoFichero() {
//   float lat = lastJsondata["lat"];
//   float lon = lastJsondata["lon"];
//   float alt = lastJsondata["alt"];
//   char filename[13] = "00000000.csv";
//   generarFileName(lastJsondata, filename);
//   //Serial.println(filename);
//   //String filename= "2019_09_23_11_43_39.csv";
//   ficheroDatos = SD.open(filename, O_CREAT | O_WRITE);
//   //ficheroDatos = SD.open("hola.txt", O_CREAT | O_WRITE);
//   //activa guardado en SD durante la llegada de datos del licor
//   ficheroDatos.println("Fecha y hora del punto:," + datetimestr.date + " " );//ficheroDatos.flush();
//   ficheroDatos.println("Latitud," + String(lat, 5));  ficheroDatos.flush();
//   ficheroDatos.println("Longitud," + String(lon, 5));  ficheroDatos.flush();
//   ficheroDatos.println("Altitud," + String(alt, 1));   ficheroDatos.flush();
//   ficheroDatos.println(cabeceraCsv(lastJsondata));     ficheroDatos.flush();

// }

/*
   genera nombre de archivo
   TODO: futura fecha de GPS.
*/
void generarFileName(DynamicJsonDocument &lastJsondata, char* dest) {
  const char* date = lastJsondata["date"];
  const char* gpsTime = lastJsondata["gpsTime"];
  dest[0] = date[5];
  dest[1] = date[6];
  dest[2] = date[8];
  dest[3] = date[9];
  dest[4] = gpsTime[0];
  dest[5] = gpsTime[1];
  dest[6] = gpsTime[3];
  dest[7] = gpsTime[4];
  dest[8] = '.';
  dest[9] = 'c';
  dest[10] = 's';
  dest[11] = 'v';
  dest[12] = '\0';
//  Serial.println(dest);
}
//------------------------------------------------------------FIN FICHEROS-------------------------------------------------------------

//---------------------------------------------------------GENERAR TRAMA -------------------------------------------------
/*
   devuelve una trama con un formato especificado
   con o si dato de tiempo.
*/
String generateTramaData(DynamicJsonDocument &doc, bool writeTime ) {
  //const char* json = "{\"time\":396.837,\"tempAmb\":27.2,\"humAmb\":53.7,\"dis\":17,\"caudal\":0.000,\"licor\":{\"model\":\"li850\",\"celltemp\":\"5.14571e1\",\"cellpres\":\"9.78824e1\",\"co2\":\"4.73465e2\",\"co2abs\":\"9.3194065e-2\",\"h2o\":\"2.20836e1\",\"h2oabs\":\"1.1426472e-1\",\"h2odewpoint\":\"1.86805e1\",\"ivolt\":\"1.1445319e1\",\"flowrate\":\"0000.000\"},\"ana\":{\"A0\":0.000,\"A2\":0.000,\"A3\":0.000}}";

  float tiempo = doc["time"]; // 396.837

  float tempAmb = doc["tempAmb"]; // 27.2
  float humAmb = doc["humAmb"]; // 53.7
  float dis = doc["dis"]; // 17

  

  JsonObject licor = doc["licor"];
  String licor_model = licor["model"]; // "li850"
  String licor_celltemp = licor["celltemp"]; // "5.14571e1"
  String licor_cellpres = licor["cellpres"]; // "9.78824e1"
  String licor_co2 = licor["co2"]; // "4.73465e2"
  String licor_co2abs = licor["co2abs"]; // "9.3194065e-2"

  String licor_h2o = "0.000000000e1";
  String licor_h2oabs = "0.000000000e1";
  String licor_h2odewpoint = "0.000000000e1";
  if (licor_model == "li850") {
    licor_h2o = (const char*)(licor["h2o"]); // "2.20836e1"
    licor_h2oabs = (const char*)(licor["h2oabs"]); // "1.1426472e-1"
    licor_h2odewpoint = (const char*)(licor["h2odewpoint"]); // "1.86805e1"
  }
  String licor_ivolt = licor["ivolt"]; // "1.1445319e1"
  String licor_flowrate = licor["flowrate"]; // "0"
  String licor_vout0 = licor["vout0"]; // "0"
  String licor_vout1 = licor["vout1"]; // "0"

  JsonObject ana = doc["ana"];
  float caudal = ana["sccm"]; // 0
  float ana_A0 = ana["A0"]; // 0
  float ana_A2 = ana["A2"]; // 0
  float ana_A3 = ana["A3"]; // 0

  float alt = doc["alt"]; // 360.5
  String analogico = "[A00:" + String(ana_A0) + ",A01:" + String(caudal) + ",A02:" + String(ana_A2) + ",A03:" + String(ana_A3) + "]";

  String licortrama;
  //licortrama.reserve(160);                              //Arduino tiene un problema con los String.
  //NOTA: reservar poca memoria, la justa para el tamaño de string determinado
  //licortrama= "[<li850><data>,celltemp:0.0000000000e1,cellpres:0.0000000000e1,co2:0.0000000000e1,h2o:0.0000000000e1,ivolt:0.0000000000e1,anaco2:0.0000000000e1,anah2o:0.0000000000e1]";

  if (licorModel.compareTo("li850") == 0) {
    licortrama = "[<li850><data>,celltemp:" + licor_celltemp + ",cellpres:" + licor_cellpres + ",co2:" + licor_co2 + ",h2o:" + licor_h2o + ",ivolt:" + licor_ivolt + ",anaco2:" + licor_vout0 + ",anah2o:" + licor_vout1 + "]";
  }
  if (licorModel.compareTo("li820") == 0) {
    licortrama = "[<li820><data>,celltemp:" + licor_celltemp + ",cellpres:" + licor_cellpres + ",co2:" + licor_co2 + ",ivolt:" + licor_ivolt + ",anaco2:" + licor_vout0 + ",anah2o:" + licor_vout1 + ",zz:0.0]";
  }

  //Serial.println(licortrama);
  String TRAMADATA;
  //TRAMADATA.reserve(280);
  TRAMADATA = "[";
  TRAMADATA.concat("TAM:" + String(tempAmb, 2) + ",");
  TRAMADATA.concat("HAM:" + String(humAmb, 2) + ",");
  TRAMADATA.concat("DIS:" + String(dis, 2) + ",");
  TRAMADATA.concat("ANA:" + analogico + ",");
  TRAMADATA.concat("LIC:" + licortrama);
  if (writeTime) {
    TRAMADATA.concat(",TIME:" + String(tiempo, 3))  ;
  }

  TRAMADATA.concat(",alt:" + String(alt, 1) + "]");
  //Serial.println(TRAMADATA);
  return TRAMADATA;
}

//---------------------------------------------------- FIN GENERAR TRAMA -------------------------------------------------

/*
   devuelve una linea csv por muestra del licor con todos los parámetros.
*/
String jsonTOcsvline(DynamicJsonDocument &doc ) {
  String csvline;
  //csvline.reserve(190);                                 //Arduino tiene un problema con los String.
  //NOTA: reservar poca memoria, la justa para el tamaño de string determinado
  float tiempo = doc["time"]; // 396.837
  float tempAmb = doc["tempAmb"]; // 27.2
  float humAmb = doc["humAmb"]; // 53.7
  float dis = doc["dis"]; // 17
  
  float lat = doc["lat"];
  float lon = doc["lon"];
  float alt = doc["alt"];
  String date = doc["date"];
  String gpsTime = doc["gpsTime"];

  JsonObject licor = doc["licor"];
  String licor_model = licor["model"]; // "li850"
  String licor_celltemp = licor["celltemp"]; // "5.14571e1"
  String licor_cellpres = licor["cellpres"]; // "9.78824e1"
  String licor_co2 = licor["co2"]; // "4.73465e2"
  String licor_co2abs = licor["co2abs"]; // "9.3194065e-2"
  String licor_h2o = "0.000000000e1";
  String licor_h2oabs = "0.000000000e1";
  String licor_h2odewpoint = "0.000000000e1";
  float licor_vout1=0.0;
  if (licorModel.compareTo("li850") == 0) {
    licor_h2o = (const char*)licor["h2o"]; // "2.20836e1"
    licor_h2oabs = (const char*)licor["h2oabs"]; // "1.1426472e-1"
    licor_h2odewpoint = (const char*)licor["h2odewpoint"]; // "1.86805e1"
    licor_vout1 = licor["vout1"]; // h2o Analogico
  }
  String licor_ivolt = licor["ivolt"]; // "1.1445319e1"
  String licor_flowrate = licor["flowrate"]; // "0"
  float licor_vout0 = licor["vout0"]; // "0"


  JsonObject ana = doc["ana"];
  float ana_A0 = ana["A0"]; // 0
  float caudal = doc["caudal"]; // 0
  float ana_A2 = ana["A2"]; // 0
  float ana_A3 = ana["A3"]; // 0
  gpsTime.replace(".", ":");

  csvline.concat(date + " " + gpsTime  + ",");
  csvline.concat( String(tiempo, 3) + ",");
  csvline.concat( String(licor_co2.toFloat(), 2) + ",");
  csvline.concat( String(licor_vout0, 2) + ",");
  csvline.concat( String(licor_co2abs.toFloat(), 2) + ",");
  csvline.concat( String(licor_celltemp.toFloat(), 1) + ",");
  csvline.concat( String(licor_cellpres.toFloat(), 1) + ",");
  csvline.concat( String(tempAmb, 2) + ",");
  csvline.concat( String(humAmb, 2) + ",");
  csvline.concat( String(dis, 2) + ",");
  csvline.concat( String(caudal, 2) + ",");
  csvline.concat( String(lat, 5) + ",");
  csvline.concat( String(lon, 5) + ",");
  csvline.concat( String(alt, 1) + ",");
  csvline.concat( String(licor_ivolt.toFloat(), 2) + ",");
  csvline.concat( String(licor_flowrate.toFloat(), 2) + ",");
  csvline.concat( String(ana_A0, 2) + ",");
  csvline.concat( String(ana_A2, 2) + ",");
  csvline.concat( String(ana_A3, 2));

  if (licor_model == "li850") {

    csvline.concat( "," + String(licor_h2o.toFloat(), 2));
    csvline.concat( "," + String(licor_vout1, 2));
    csvline.concat( "," + String(licor_h2oabs.toFloat(), 2));
    csvline.concat( "," + String(licor_h2odewpoint.toFloat(), 2));
  }

  USB_PORT.println(csvline);
  return csvline;
}

String cabeceraCsv(DynamicJsonDocument doc) {
  JsonObject licor = doc["licor"];
  String licor_model = licor["model"]; // "li850"
  if (licor_model == "li850") {
    return "Fecha,Tiempo de muestra (seg),CO2(ppm),CO2ana(ppm),CO2abs,T.Celda (ºC),P.Celda,Tem.Amb(ºC) ,Hum.Amb(%),Altura Campana(cm),Caudal(sccm),Latitud,Longitud, Altura(m),Bateria (V),Flowrate,Ana0(mV),Ana2(mV),Ana3(mV),H2O,H2Oana,H2Oabs,H2Odewpoint";
  }
  else if (licor_model == "li820") {
    return "Fecha,Tiempo de muestra (seg),CO2(ppm),CO2ana(ppm),CO2abs,T.Celda (ºC),P.Celda,Tem.Amb(ºC) ,Hum.Amb(%),Altura Campana(cm),Caudal(sccm),Latitud,Longitud, Altura(m),Bateria (V),Flowrate,Ana0(mV),Ana2(mV),Ana3(mV)";
  } else {
    return "¡¡MODELO DE SENSOR NO RECONOCIDO!!";
  }
}


//-----------------------------------------------------------------------FIN TRAMAS Y CSV-----------------------------------

/*
   imprime bufferCO2
*/
// void printbuffer()
// {
//   char separator = '|';
//   for (decltype(bufferCO2)::index_t i = 0; i < bufferCO2.size(); i++) {

//     USB_PORT.print(bufferCO2[i]);
//     USB_PORT.print(separator);
//   }
//   Serial.println();
// }


//------------------------------------------------------------CALCULOS MATEMATICOS--------------------------------------------
/*
   Calculos matematicos sobre buffer circular.
   calcula media,stdev,pendiente(flujo), terminoindependiente
   retorna struct con CalculoMath.
*/
// struct CalculoMath calcularMath() {
//   double pend_termIndp[] = {0.0, 0.0};
//   CalculoMath calculos;
//   long start = millis();
//   int nElementos = bufferCO2.size();
//   USB_PORT.println(nElementos);

//   if ( nElementos > 1 ) {
//     LinearRegression lr = LinearRegression(bufferTiempo.first(), bufferTiempo.last() );
//     Average<float> ave(nElementos);
//     for (int i = 0; i <  nElementos ; i++) {          // calcula siempre sobre el buffer circular

//       lr.learn(bufferTiempo[i], bufferCO2[i]);
//       ave.push(bufferCO2[i]);
//     }
//     lr.getValues(pend_termIndp);
//     float correlation = lr.correlation();
//     if (!isnan(correlation)) {                                //correlación a veces da un valor nan (division por 0)-> no cambiamos el anterior
//       calculos.r2 = lr.correlation() * lr.correlation();      //para optener R2
//     }
//     calculos.mean = ave.mean();
//     calculos.stdev = ave.stddev();
//     calculos.fluxPend = (float)pend_termIndp[0];
//     calculos.termIndep = (float)pend_termIndp[1];

//     if (inicio && updateMediaStdev ) {                                          // Para mostrar linea 3 y 4 guarda en var globales los calculos
//       media = calculos.mean;
//       stdev = calculos.stdev;
//     }
//     if (acumular && updateFlujoR2) {
//       flujo = calculos.fluxPend;
//       r2 = calculos.r2;
//     }

//   }
//   USB_PORT.println(calculos.mean);
//   USB_PORT.println(calculos.stdev);
//   USB_PORT.println(calculos.fluxPend);
//   USB_PORT.println(calculos.r2);
//   return calculos;
// }

/*
   genera nuevo fichero con su nombre, datos iniciales y cabecera
*/
// void imprimeCalculos() {
//   ficheroDatos.println(); ficheroDatos.flush();
//   ficheroDatos.println("To:," + String(t0, 3) + ",CO2 t0," + String(co2t0, 2) ); ficheroDatos.flush();
//   ficheroDatos.println("Media," + String(media, 2) + ",Stdev" + String(stdev, 2));  ficheroDatos.flush();
//   ficheroDatos.println("Ti:," + String(ti, 3) + ",CO2 t0," + String(co2ti, 2) ); ficheroDatos.flush();
//   ficheroDatos.println("Flujo," + String(flujo, 2) + ",ERRQ," + String(r2, 2));  ficheroDatos.flush();
//   ficheroDatos.println("Tf:," + String(tf, 3) + ",CO2 tf," + String(co2tf, 2) ); ficheroDatos.flush();
// }
