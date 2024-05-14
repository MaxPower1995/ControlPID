#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <Wire.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define LEFT_PWM_1 25  // Pin PWM para controlar OUT1
#define RIGHT_PWM_1 33// Pin PWM para controlar OUT2
#define ENABLE_PELTIER_1 32  //
#define LEDS_CAM_1 22  // 

#define LEFT_PWM_2 10  //
#define RIGHT_PWM_2 12  //
#define ENABLE_PELTIER_2 13  //
#define LEDS_CAM_2 23  //

#define TEMP_SENSOR_PIN 4  // Pin para el sensor de temperatura
#define DESVIO 1.1         // Desvio de temperatura porcentual

#define THINGSBOARD_SERVER "panel.akiot.es"
#define TOKEN "camaraLaboratorio1"

#define CAL_SUP -0.5625 //mide con - 0.0625 de error, se ajusto 31/1/24
#define CAL_INF -0.2500 //mide con - 0.2500 de error, se ajusto 1/2/24

#define IP "192.168.0.177"
#define MAC "E8:6B:EA:CF:D0:A4"
#define HARDWARE "ESP32"
#define SOFTWARE "2.1"

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
ThingsBoard tb(espClient);

/* const char* ssids[] = {"MOTOMAX", "Edificio INNOVA", "MOTO"};
const char* passwords[] = {"1234567890", "Mexico932", "1234567890"}; */
int networkIndex = 0;
unsigned long lastRestartTime = 0;
unsigned long lastSend = 0;

#define SSID "Edificio INNOVA"
#define PASS  "Mexico932"

/* const char* SSID = "MOTO";
const char* PASS = "1234567890"; */

const int intentoMaximoConexion = 10;
bool conectado_al_thingsboard = false;

//#############################################
//#############################################
double setpoint = 25;  // Temperatura objetivo

double tempCamSup, tempCamInf;  // Temperatura de la camara
double input1, output;
double input, output_1_frio, output_1_calor, output_2_frio, output_2_calor;
double output_1, output_2;

//Con temperatura controlada en el laboratorio, esto produce una
//respuesta rapida y aceptable para 25 grados sin desvio aceptable
//double Kp = 150, Ki = 10, Kd = 0; 

double Kp_1_frio = 450, Ki_1_frio = 25, Kd_1_frio = 0;
double Kp_1_calor = 150, Ki_1_calor = 10, Kd_1_calor = 0;
double Kp_2_frio = 150, Ki_2_frio = 10, Kd_2_frio = 0;
double Kp_2_calor = 150, Ki_2_calor = 10, Kd_2_calor = 0;

double Kp = 150, Ki = 10, Kd = 0;  // Sintoniza estos valores según sea necesario
PID myPID(&tempCamSup, &output, &setpoint, Kp, Ki, Kd, DIRECT); //Control PID para Calor

//Como los peltiers tienen distinta respuesta para calentar y enfriar, tengo que hacer dos PID por camara
//Camara Superior
PID myPID_1_frio(&tempCamSup, &output_1_frio, &setpoint, Kp_1_frio, Ki_1_frio, Kd_1_frio, DIRECT);
PID myPID_1_calor(&tempCamSup, &output_1_calor, &setpoint, Kp_1_calor, Ki_1_calor, Kd_1_calor, DIRECT);
//Camara Inferior
PID myPID_2_frio(&tempCamInf, &output_2_frio, &setpoint, Kp_2_frio, Ki_2_frio, Kd_2_frio, DIRECT);
PID myPID_2_calor(&tempCamInf, &output_2_calor, &setpoint, Kp_2_calor, Ki_2_calor, Kd_2_calor, DIRECT);

void wifi();
void wifiOTA();
void otaSetup();
bool conecto_al_thingsboard();
void direccionesSensores();
void sensores(double* temperatura1,double* temperatura2);
void print_current_time();

IPAddress local_IP(192, 168, 0, 177);     //Camaras Laboratorio
IPAddress gateway(192, 168, 0, 1);          
IPAddress subnet(255, 255, 255, 0); 
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4); 

float pid_sup_values[50] = {0};
float pid_inf_values[50] = {0};
int pid_index = 0;
float sum_sup = 0;
float sum_inf = 0;
float average_sup = 0;
float average_inf = 0;

const char* ntpServer = "pool.ntp.org";
const int timeZone = -3;
int year = 1970;
struct tm * ti;
time_t rawtime;

//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, ntpServer, timeZone * 3600);

void setup() {
  Serial.begin(115200);
  wifiOTA();
  otaSetup();
  sensors.begin();

  pinMode(LEFT_PWM_1, OUTPUT);
  pinMode(RIGHT_PWM_1, OUTPUT);
  pinMode(ENABLE_PELTIER_1, OUTPUT);

  pinMode(LEFT_PWM_2, OUTPUT);
  pinMode(RIGHT_PWM_2, OUTPUT);
  pinMode(ENABLE_PELTIER_2, OUTPUT); 

  pinMode(LEDS_CAM_1, OUTPUT);
  pinMode(LEDS_CAM_2, OUTPUT);

  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

  myPID_1_frio.SetOutputLimits(0, 255);
  myPID_1_frio.SetMode(AUTOMATIC);

  myPID_1_calor.SetOutputLimits(0, 255);
  myPID_1_calor.SetMode(AUTOMATIC);

  myPID_2_frio.SetOutputLimits(0, 255);
  myPID_2_frio.SetMode(AUTOMATIC);

  myPID_2_calor.SetOutputLimits(0, 255);
  myPID_2_calor.SetMode(AUTOMATIC);

  //timeClient.begin();

}

void loop()
{
  if (WiFi.status() != WL_CONNECTED){
    wifiOTA();
    conectado_al_thingsboard = conecto_al_thingsboard(); 
  }
  ArduinoOTA.handle();

/*   rawtime = timeClient.getEpochTime();
  ti = gmtime (&rawtime);
  year = ti->tm_year + 1900;

  if(year < 2022){
    timeClient.update();
    Serial.println("Actualizando hora");
  } */

  sensors.requestTemperatures();
  sensores(&tempCamSup,&tempCamInf); 

  tempCamSup = tempCamSup + CAL_SUP;
  tempCamInf = tempCamInf + CAL_INF;

  Serial.print("Sensor 1:");
  Serial.println(tempCamSup,4);
  Serial.print("Sensor 2:");
  Serial.println(tempCamInf,4);

  digitalWrite(ENABLE_PELTIER_1, HIGH);
  digitalWrite(ENABLE_PELTIER_2, HIGH);

if (tempCamSup > 0){
  if(tempCamSup < setpoint ){
    myPID_1_calor.SetControllerDirection(DIRECT);
    myPID_1_calor.Compute(); // Calcula el valor de salida del PID
    Serial.println("Calentando");
    analogWrite(LEFT_PWM_1, output_1_calor);
    analogWrite(RIGHT_PWM_1, 0); 
    Serial.print("Output PID: ");
    Serial.println(output_1_calor); 
    output_1 = output_1_calor;
  }else{
    myPID_1_frio.SetControllerDirection(REVERSE);
    myPID_1_frio.Compute(); // Calcula el valor de salida del PID
    Serial.println("Enfriando");
    analogWrite(LEFT_PWM_1, 0);
    analogWrite(RIGHT_PWM_1, output_1_frio);
    Serial.print("Output PID: ");
    Serial.println(output_1_frio); 
    output_1 = output_1_frio; 
  }
}

if(tempCamInf > 0){
  if(tempCamInf < setpoint ){
    myPID_2_calor.SetControllerDirection(DIRECT);
    myPID_2_calor.Compute(); // Calcula el valor de salida del PID
    Serial.println("Calentando");
    analogWrite(LEFT_PWM_2, output_2_calor);
    analogWrite(RIGHT_PWM_2, 0); 
    Serial.print("Output PID: ");
    Serial.println(output_2_calor); 
    output_2 = output_2_calor;
  }else{
    myPID_2_frio.SetControllerDirection(REVERSE);
    myPID_2_frio.Compute(); // Calcula el valor de salida del PID
    Serial.println("Enfriando");
    analogWrite(LEFT_PWM_2, 0);
    analogWrite(RIGHT_PWM_2, output_2_frio);
    Serial.print("Output PID: ");
    Serial.println(output_2_frio); 
    output_2 = output_2_frio;
  }
}


if(tempCamSup < setpoint*1.01  && tempCamSup > setpoint*0.99){
  digitalWrite(LEDS_CAM_1, HIGH);
}else{
  digitalWrite(LEDS_CAM_1, LOW);
}

if(tempCamInf < setpoint*1.01  && tempCamInf > setpoint*0.99){
  digitalWrite(LEDS_CAM_2, HIGH);
}else{
  digitalWrite(LEDS_CAM_2, LOW);
}

if(millis() - lastSend >= 60000){
  if(conectado_al_thingsboard == true){
    Serial.println("TB OK");
    if(tempCamSup > 0){
      tb.sendTelemetryFloat("AK-057", tempCamSup);
    }
    if(tempCamInf > 0){
      tb.sendTelemetryFloat("AK-027", tempCamInf);
    }
    tb.sendTelemetryFloat("Setpoint", setpoint);
    tb.sendTelemetryFloat("Potencia AK-057", (output_1*100)/255);
    tb.sendTelemetryFloat("Potencia AK-027", (output_2*100)/255);
    tb.sendTelemetryFloat("Promedio AK-057", average_sup);
    tb.sendTelemetryFloat("Promedio AK-027", average_inf);
    lastSend = millis();
  }else{
    conectado_al_thingsboard = conecto_al_thingsboard();
  }
}

/*   print_current_time();

  // Si son las 7:30 AM
  if (ti->tm_hour == 7 && ti->tm_min == 30) {
    ESP.restart(); // Reiniciar el dispositivo
  } */

  // Almacenar el valor del PID en el array
  pid_sup_values[pid_index] = output_1; 
  pid_inf_values[pid_index] = output_2;

  // Incrementar el índice y reiniciarlo a 0 si llega a 50
  pid_index = (pid_index + 1) % 50;

  // Calcular la media de los valores en el array
for (int i = 0; i < 50; i++) {
  sum_sup += pid_sup_values[i];
  sum_inf += pid_inf_values[i];
}

average_sup = sum_sup / 50;
average_inf = sum_inf / 50;

  // Si han pasado 4/2 horas desde la última vez que se reinició el dispositivo
  if (millis() - lastRestartTime >= 14400000/2) {
    lastRestartTime = millis(); // Actualizar la última vez que se reinició el dispositivo
    ESP.restart(); // Reiniciar el dispositivo
  }

//SE esta reiniciando a las 10 muestras aprox!!!
/* 
// Si la media es mayor a 200, reiniciar el dispositivo
if (average_sup > 200 || average_inf > 200) {
  ESP.restart();
} */

  delay(1000);
}

/* void wifi(){
  int intentos = 0;

  WiFi.begin(ssids[networkIndex],passwords[networkIndex]);
  Serial.print(F("Conectandose a la red "));
  Serial.print(ssids[networkIndex]);
  
  while (WiFi.status() != WL_CONNECTED && intentos < intentoMaximoConexion){
    delay(500);
    Serial.print(F("."));
    intentos++;
  }
  
  Serial.println(F(" "));

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("Se ha conectado a su red");
    Serial.print(F("Direccion de IP: "));
    Serial.println(WiFi.localIP());
  }else{
    Serial.println("NO Se ha conectado a su red");
  }

  networkIndex = (networkIndex + 1) % 3;
} */

void wifiOTA(){
/* Esta funcion de WiFi prueba conexion durante 10 intentos
si pasado estos intentos no se conecta sigue, esto es una
funcion No bloqueante a medias (5seg), lo bueno que si
se cae la red no queda en un loop infinito y los equipos
pueden seguir controlando normalmente.
*/
  int intentos = 0;

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print(F("Conectandose a la red "));
  Serial.print(SSID);
  
  while (WiFi.status() != WL_CONNECTED && intentos < intentoMaximoConexion){
    delay(500);
    Serial.print(F("."));
    intentos++;
  }
  
  Serial.println(F(" "));

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("Se ha conectado a su red");
    Serial.print(F("Direccion de IP: "));
    Serial.println(WiFi.localIP());
    Serial.print("Dirección MAC: ");
  Serial.println(WiFi.macAddress());
  }else{
    Serial.println("NO Se ha conectado a su red");
  }

  Serial.println();

}

bool conecto_al_thingsboard(){
  //creo que esto se puede mandar directamente al loop!
  //deberia probar si ponen mal el TOKEN o SERVER
  //deberia ver que pasa si se corta el wifi en el medio !
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("No hay conexion WiFi");
    wifiOTA();
    return false;
  }else{
    if(!tb.connected()){
      Serial.print(F("Conectandose a : "));
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(F(" con el Token "));
      Serial.println(TOKEN);
        if(!tb.connect(THINGSBOARD_SERVER,TOKEN)){
          Serial.println(F("Fallo la conexion al servidor"));
          return false;
        }else{
          Serial.println(F("Conexion exitosa al servidor"));
          return true;
        }
      }else{
        return true;
      }
  }
}

void direccionesSensores(){
  byte addr[8];  
  Serial.println("Obteniendo direcciones:");
    while (oneWire.search(addr)) {  
    Serial.print("Address = ");
      for( int i = 0; i < 8; i++) {
        Serial.print(" 0x");
        Serial.print(addr[i], HEX);
      }
      Serial.println();
    }
  Serial.println();
}

void sensores(double* temperatura1,double* temperatura2){
  DeviceAddress direccion1 = {0x28, 0x1B, 0x5E, 0x95, 0xF0, 0x1, 0x3C, 0xBE};
  DeviceAddress direccion2 = {0x28, 0xA9, 0xC, 0x95, 0xF0, 0x1, 0x3C, 0xC1};

  sensors.requestTemperatures();
  *temperatura1 = sensors.getTempC(direccion1);
  *temperatura2 = sensors.getTempC(direccion2); 

/*   Serial.println(F("Mediciones de Temperatura: ")); */

/*   Serial.print(F("Sensor 1: "));
  if(*temperatura1 == -127){
    Serial.println(F("Sonda Desconectada"));
  }else{
    Serial.print(*temperatura1,4);
    Serial.println(F(" °C"));
  }

  Serial.print(F("Sensor 2: "));
  if(*temperatura2 == -127){
    Serial.println(F("Sonda Desconectada"));
  }else{
    Serial.print(*temperatura2,4);
    Serial.println(F(" °C"));
  } */

/*   Serial.println(); */
}

void print_current_time(){
    //timeClient.update();

 /*    rawtime = timeClient.getEpochTime();
    ti = gmtime (&rawtime); */

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", ti);

    Serial.print("Hora Actual: ");
    Serial.println(strftime_buf);
}

void otaSetup(){
  //Esto es para hacerle configuraciones adicionales al OTA, quiza la unica piola es
  //setearle un password para que no se conecte cualquiera y ponga cualquier codigo

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

