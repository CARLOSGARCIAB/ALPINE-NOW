#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "SparkFunBME280.h"
#include <analogWrite.h>
//Añadidos
#include <LedRGB.h>



//Variables lectura  BME280 y Credenciales WIFI
  float temperature, altitude, pressure, humidity,latitud,longitud;
  float Boton,LedRojo;
  const char* ssid = "carlos";
  const char* password =  "carloswifi";
  


// sensor variables
  BME280 mySensor;

// The TinyGPSPlus object
  TinyGPSPlus gps;

// The serial connection to the GPS device
  SoftwareSerial ss(18, 19);
  
//Añadidos

  LedRGB LED(17, 16, 5, CC);
  const int pulsadorGPIO = 15;
  
  
  bool estadoBoton =  false; 
  
void setup()
{
  Serial.begin(115200);
  //-----------------------------------------------------------------------------------
  // GPS Setup
  //-----------------------------------------------------------------------------------
  ss.begin(9600);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

  //-----------------------------------------------------------------------------------
  // BME280 sensor Setup
  //-----------------------------------------------------------------------------------
  Wire.begin();

  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  pinMode(4, OUTPUT);
  //-----------------------------------------------------------------------------------
  // Wifi Setup
  //-----------------------------------------------------------------------------------
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: ");
  Serial.println(WiFi.localIP()); //print LAN IP

  //Añadidos
  LED.test(); // ENcendido de los 3 leds
   pinMode(pulsadorGPIO, INPUT_PULLUP);    
}

void loop()
{
  
  
  if ((ss.available() > 0) && (gps.encode(ss.read())) && (WiFi.status() == WL_CONNECTED) ) 
  {
    // Aquí puedes acceder a la latitud y longitud si están disponibles
    //-----------------------------------------------------------------------------------
    // BME280 SENSOR READING
    // 3.3V
    // SCL: G22
    // SDA: G21
    //-----------------------------------------------------------------------------------
     humidity =    (mySensor.readFloatHumidity());
     pressure =    (mySensor.readFloatPressure());
     altitude =    (mySensor.readFloatAltitudeMeters());
     temperature = (mySensor.readTempC());
     
    //-----------------------------------------------------------------------------------
    // GPS reading
    // 3.3V
    // tx -> rx
    // rx -> tx
    //-----------------------------------------------------------------------------------
    if (gps.location.isValid()) 
    {
       latitud = gps.location.lat();
       longitud = gps.location.lng();

      // Ahora puedes hacer lo que necesites con latitud y longitud
      Serial.print(F("Latitud: "));
      Serial.println(latitud, 6);
      Serial.print(F("Longitud: "));
      Serial.println(longitud, 6);
      Serial.print("Altitud: ");
      Serial.println(mySensor.readFloatAltitudeMeters(), 1);
      Serial.print("Humedad: ");
      Serial.println(mySensor.readFloatHumidity(), 0);
      Serial.print("Presión: ");
      Serial.println(mySensor.readFloatPressure(), 0);
      Serial.print("Temperatura: ");
      Serial.println(mySensor.readTempC(), 2);

    }
     estadoBoton = digitalRead(pulsadorGPIO);   
  // Leemos el boton y leemos Temp
  if (estadoBoton == HIGH) 
  {      
       Boton=0;
  }  
  else 
  {      
        Boton=1;
  }

  if ( temperature < 22 )
  {
   LED.ponerColor(255, 0, 0);
   LedRojo = 1;
  }
  else
  {
     LED.ponerColor(0,255, 0);
     LedRojo = 0;
  }
    //-----------------------------------------------------------------------------------
    // OXIGEN DISTRIBUTION (SERVO MOVEMENT) // LED TEMPERATURA / BOTON
    // El negro o el marrón: se conecta a tierra también denominada: ground, GND. Es decir al polo negativo de la batería.
    // Rojo: se conecta al polo positivo de la batería.
    // Blanco o naranja o amarillo: es el cable de control de señal del servo. Se conecta al controlador del funcionamiento del servo, por ejemplo un pin PWM~ de salida digital de la tarjeta arduino.
    //-----------------------------------------------------------------------------------
  
      double alti = mySensor.readFloatAltitudeMeters();    
      int grados = (int)AltitudAGradosServo(alti, 554, 558, 0, 180);
      analogServo(4, grados);  
     
      
    //-----------------------------------------------------------------------------------
    // CREATE JSON
    //-----------------------------------------------------------------------------------
    StaticJsonDocument<200> doc;
    
    //sensor    
    doc["temperature"] = temperature;
    doc["pressure"] = pressure;
    doc["altitude"] = altitude;
    doc["humidity"] = humidity;
    //gps
    doc["latitud"] =latitud;
    doc["longitud"] =longitud;
    doc["Boton"] = Boton;
    doc ["LedRojo"]= LedRojo; // 1 si esta ojo , 0 si está Verde
    doc["Regulador_Oxigeno"]= grados;
    String json_string_pretty;
    serializeJsonPretty(doc, json_string_pretty);
    
  //-----------------------------------------------------------------------------------
  // HTTP CONNECTION
  //-----------------------------------------------------------------------------------
    HTTPClient http;  
    http.begin("http://192.168.226.194:5000/sensor_values");   
    http.addHeader("Content-Type", "application/json"); 
  //-----------------------------------------------------------------------------------
  // HTTP POST
  //-----------------------------------------------------------------------------------
    int httpResponseCode = http.POST(json_string_pretty);
    if (httpResponseCode > 0) 
    {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } 
    else 
    {
      Serial.print("Error on sending POST Request: ");
      Serial.println(httpResponseCode);
    }
    
  http.end();
    
    } 
   
 }


//-----------------------------------------------------------------------------------
// funcion Actuador en relacion a la altitud
//-----------------------------------------------------------------------------------



float AltitudAGradosServo(double x, double in_min, double in_max, double out_min, double out_max) 
{
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
