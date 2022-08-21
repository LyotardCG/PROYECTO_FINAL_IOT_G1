#include <ESP8266WiFi.h> //libreria que proporciona las rutinas especificas Wifi de ESP8266
#include <PubSubClient.h>

/*************************Variables del DHT22**********************************/
#include <DHT.h> //Cargamos la librería DHT
#define DHTTYPE DHT22 //Definimos el modelo del sensor DHT22
#define DHTPIN 4 // Se define el pin 4 (D2) del ESP8266 para conectar el pin de datos del DHT22
DHT dht(DHTPIN, DHTTYPE, 22);

/*************************Variables del sensor de humedad**********************/
#define sensorHumedad A0 //Conectamos el sensor de humedad al puerto analogico
//definimos una variable para almacenar las lecturas del sensor de humedad
float hum_suelo = 0;
/*************************Para la bomba de agua********************************/
int bombaAgua = 5; //indica el pin del rele al arduino
/************************Para los 2 LED que usaremos***************************/
int LED1 = 14; //estara encendido cuando se este activado la bomba de agua
int LED2 = 12; //si la humedad del suelo esta demasiado baja (<45%)

/********************Parametros de conexion Wifi*******************************/
// Depende de cada maquina
const char* ssid = "Sergio"; //nombre de la red con la que trabajamos
const char* password = "07503108"; //contraseña del la red con la que trabajamos
/*************************MQTT*************************************************/
const char* mqtt_server = "192.168.0.5";
String mensaje; //para almacenar el topic
String inform; //para almacenar el payload


// Creamos unas variables para almacenar los mensajes
char dht22tem[15], dht22hum[15], sensorhum[15];

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

void setup_wifi() {
  delay(10);
  // Empezamos conectándonos a una red WiFi
  Serial.println();
  Serial.print("Conectado a ");
  Serial.println(ssid);

  //establecemos el inicio de la conexión con los credenciales del Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { //Mientras no este conectado imprimimos un punto
    delay(500);  //esperamos 0.5 segundos
    Serial.print(".");
  }
  Serial.println("");
  //despues de lograrse la conexion imprimos un mensaje
  Serial.println("Conectado a la red WiFi con dirección IP: ");
  Serial.println(WiFi.localIP()); //Imprimimos la dirección IP de la red Wifi


}
//Funcion para comprobar si recibe el topic
void callback(char* topic, byte* payload, unsigned int length) {
  mensaje = topic;
  inform = "";
  Serial.print("El mensaje llegó [");
  Serial.print(mensaje);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    inform += (char)payload[i]; //guardamos el payload en un string
  }
  Serial.print(inform);
  Serial.println();

  //Control de LEDs por Node-red
  if (mensaje == "led1") {
    if (inform == "true") {
      digitalWrite(LED1, HIGH);
    }
    else {
      digitalWrite(LED1, LOW);
    }
  }

  if (mensaje == "led2") {
    if (inform == "true") {
      digitalWrite(LED2, HIGH);
    }
    else {
      digitalWrite(LED2, LOW);
    }
  }

  //Control del riego
}

void reconnect() {
  // Loop hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado");
      client.publish("led1", "Conectado");
      client.publish("led2", "Conectado");
      client.publish("dht22-tem", "Conectado");
      client.publish("dht22-hum", "Conectado");
      client.publish("sensor-hum_suelo", "Conectado");
      client.publish("control_riego", "Conectado");

      //nos suscribimos a los topic
      client.subscribe("led1");
      client.subscribe("led2");
      client.subscribe("dht22-tem");
      client.subscribe("dht22-hum");
      client.subscribe("sensor-hum_suelo");
      client.subscribe("control_riego");

    } else {
      Serial.print("Conexion fallida, rc=");
      Serial.print(client.state());
      Serial.println("....pruebe de nuevo dentro de 5 segundos....");
      delay(5000);  //Esperamos 5 segundos
    }
  }
}

void setup() {
  //pinMode(BUILTIN_LED, OUTPUT);
  /*****************Para la bomba de agua**********************/
  //pinMode(bombaAgua, OUTPUT); //establecemos el pin 8 como salida
  /************************************************************/
  /*****************Para los LED**********************/
  pinMode(LED1, OUTPUT); //establecemos el pin 14 como salida
  pinMode(LED2, OUTPUT); //establecemos el pin 12 como salida
  /************************************************************/

  dht.begin(); //permite configurar el pin del sensor como INPUT_PULLUP
  pinMode(sensorHumedad, INPUT); //definimos el pin del sensor como INPUT
  Serial.begin(115200); //se inicio la comunicación serial

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


}

void loop() {

  //Se lee la temperatura en Celcius y se le asigna a la variable tem
  float tem = dht.readTemperature();
  //Se lee la humedad relativa y se le asigna a la variable hum
  float hum = dht.readHumidity();  //Se lee la humedad relativa y se le asigna a hum

  // Capturamos el valor del sensor de humedad de suelo en la variable hum_suelo
  // Realizamos un pequeño calculo para transformar la lectura analogica a un rango de 0-100
  hum_suelo = (100.00 - ((analogRead(sensorHumedad) * 100) / 1024.00));
  
  /*
    // Imprimimos las lecturas del sensor DHT22
    Serial.print(F("\n ===Lecturas del DHT22==="));
    Serial.print(F("\n Temperatura (°C): "));
    Serial.print(tem);
    Serial.print(F("\n Humedad (%): "));
    Serial.print(hum);
    Serial.print("\n");
    // Imprimimos las lecturas del sensor de humedad del suelo
    Serial.print(F("\n ===Lecturas del sensor de humedad==="));
    Serial.print(F("\n Humedad del suelo (%): "));
    Serial.print(hum_suelo);
    Serial.print("\n");
  */
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  riego_automatico(tem,hum,hum_suelo);

  long now = millis();

  if (now - lastMsg >= 5000) {
    lastMsg = now;
    dtostrf(tem, 0, 2, dht22tem);
    dtostrf(hum, 0, 2, dht22hum);
    dtostrf(hum_suelo, 0, 2, sensorhum);
    client.publish("dht22-tem", dht22tem);
    client.publish("dht22-hum", dht22hum);
    client.publish("sensor-hum_suelo", sensorhum);
    delay(500);
  }
}

void riego_automatico(float tem, float hum, float hum_suelo) {
  if ((tem >= 30 && hum <= 30) || (hum <= 20) || (hum_suelo <= 20 )) {
    //el LED2 se encendera por 5 segundos antes de activarse el riego
    digitalWrite(LED1,HIGH); //encendemos el led1 mientras la bomba este encendida
    delay(5000);
    digitalWrite(LED1,LOW);
    
    digitalWrite(bombaAgua, LOW); //prender bomba de agua
    delay(5000); //la bomba estara encendidad por 5 segundos
    
    digitalWrite(LED2,HIGH); //encendemos el led 2 mientras la bomba este encendida
  }
  else if (mensaje == "control_riego") { //si presiona el boton
    if (inform == "true") {
      digitalWrite(bombaAgua, LOW); //prender bomba de agua

      digitalWrite(LED2,HIGH); //encendemos el led 2 mientras la bomba este encendida
    }
    else {
      digitalWrite(bombaAgua, HIGH); //apagar bomba de agua}
    }
  } else { //si no pasa nada de los anterior estara apagada
    digitalWrite(bombaAgua, HIGH); //apagar bomba de agua
  }
}
