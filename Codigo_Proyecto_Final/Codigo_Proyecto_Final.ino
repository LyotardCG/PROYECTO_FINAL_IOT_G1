#include <ESP8266WiFi.h> //libreria que proporciona las rutinas especificas Wifi de ESP8266
#include <PubSubClient.h> //Libreria para enviar y recibir mensajes MQTT

/*************************Variables del DHT22**********************************/
#include <DHT.h> //Cargamos la librería DHT
#define DHTTYPE DHT22 //Definimos el modelo del sensor DHT22
#define DHTPIN 4 // Se define el pin 4 (D2) del ESP8266 para conectar el pin de datos del DHT22
DHT dht(DHTPIN, DHTTYPE, 22); //Definimos que estamos trabajando con el sensor DHT22

/*************************Variables del sensor de humedad**********************/
#define sensorHumedad A0 //Conectamos el sensor de humedad al puerto analogico
//Definimos una variable para almacenar las lecturas del sensor de humedad
float hum_suelo = 0;

/*************************Para la bomba de agua********************************/
int bombaAgua = 5; //Indica el pin digital donde se conecta rele (D1)

/**********************Para el sensor ultrasonido HC-SR04**********************/
//Configuramos los pines del sensor Trigger y Echo
int PinTrig = 13; //Pin (D7) donde conectaremos el pin trigger del sensor ultrasonido
int PinEcho = 15; //Pin (D8) donde conectaremos el pin echo del sensor ultrasonido

//Creamos una variable tiempo donde almacenaremos el tiempo que le toma a la onda de sonido
//recorrer una distancia a la velocidad del sonido
float tiempo;
float distancia; //Se calcula con ayuda del tiempo

#define nLecturas 100 //Tamaño del arreglo de lecturas que crearemos
float lecturas[nLecturas]; //Arreglo donde almacenaremos las lecturas
int nActual = 0; //Indice de la lectura en la que nos encontramos

//Creamos dos variables
float distanciaLleno = 3.5; //Distancia que indica que el recipiente de agua esta lleno
float distanciaVacio = 17.5; //Distancia que indica que el recipiente de agua esta vacio
//Creamos una variable donde se asignara el porcentaje de agua
//de nuestro recipiente (el almacen de agua)
float porcentajeAgua;

/************************Para los 2 LED que usaremos***************************/
int LED1 = 14; //Se encendera cuando la bomba de agua se active y estara encendida hasta que esta se apague
int LED2 = 12; //Estara encendido cuando el porcentaje de agua de nuestro recipiente es menor al 20%

/********************Parametros de conexion Wifi*******************************/
//Depende de cada maquina
const char* ssid = "Sergio"; //Nombre de la red con la que trabajamos
const char* password = "07503108"; //Contraseña del la red con la que trabajamos

/*************************MQTT*************************************************/
const char* mqtt_server = "192.168.0.5"; //Dirección ip del dispositivo
const char* mqtt_user = "admin"; //Ingresamos un usuario
const char* mqtt_pass = "12345"; //Ingresamos una contraseña

String mensaje; //Para almacenar el topic
String inform; //Para almacenar el payload

//Creamos unas variables para almacenar los mensajes que seran posteriormente publicados
char dht22tem[15], dht22hum[15], sensorhum[15], sensorultrasonic[15];

/******************************************************************************/
// Crea una clase WiFiClient para conectarse al servidor MQTT
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0; //Definimos una variable donde se almacenara el momento en donde se envio el ultimo mensaje

/******************************************************************************/
//Asiganamos una variable para almacenar el estado de la bomba
String estadoBomba = "0";

/******************************************************************************/
void setup_wifi() {
  delay(10);
  // Empezamos conectándonos a una red WiFi
  Serial.println();
  Serial.print("Conectado a ");
  Serial.println(ssid);

  //Establecemos el inicio de la conexión con los credenciales del Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { //Mientras no este conectado imprimimos un punto
    delay(500);  //Esperamos 500 milisegundos
    Serial.print(".");
  }
  Serial.println("");
  //Despues de lograrse la conexion imprimos un mensaje
  Serial.println("Conectado a la red WiFi con dirección IP: ");
  Serial.println(WiFi.localIP()); //Imprimimos la dirección IP de la red Wifi
}

/******************************************************************************/
//Funcion para comprobar si recibe el topic
void callback(char* topic, byte* payload, unsigned int length) {
  mensaje = topic; //Almacenamos el topic en la variable mensaje
  inform = ""; //String donde se almacenara el payload
  //Comentaremos los siguientes mensajes
  //Serial.print("El mensaje llegó [");
  //Serial.print(mensaje); //imprimimos el mensaje
  //Serial.print("] ");

  for (int i = 0; i < length; i++) {
    inform += (char)payload[i]; //Guardamos el payload en un string
  }
  //Serial.print(inform); //Imprimos el string que contiene al payload
  //Serial.println();
}

/******************************************************************************/
void reconnect() { //Funcion para reconectarnos
  // Loop hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT...");
    if (client.connect("ESP8266Client",mqtt_user, mqtt_pass)) { //Si estamos conectados, entonces...
      //Publicamos en cada uno de los topic el mensaje Conectado
      Serial.println("Conectado");
      client.publish("dht22-tem", "Conectado");
      client.publish("dht22-hum", "Conectado");
      client.publish("sensor-hum_suelo", "Conectado");
      client.publish("sensor-ultrasonido", "Conectado");
      client.publish("control_riego", "Conectado");

      //Nos suscribimos a los topicos
      client.subscribe("dht22-tem");
      client.subscribe("dht22-hum");
      client.subscribe("sensor-hum_suelo");
      client.subscribe("sensor-ultrasonido");
      client.subscribe("control_riego");

    } else { //si falla la conexión se imprime un mensaje
      Serial.print("Conexion fallida, rc=");
      Serial.print(client.state()); //Se imprime la condicion del cliente
      Serial.println("....pruebe de nuevo dentro de 5 segundos....");
      delay(5000);  //Esperamos 5 segundos
    }
  }
}

/******************************************************************************/
void setup() {
  /*****************Para la bomba de agua**********************/
  pinMode(bombaAgua, OUTPUT); //Establecemos el pin 8 como salida

  /************Para el sensor ultrasonido HC-SR04**************/
  // Ponemos el pin Trig en modo salida
  pinMode(PinTrig, OUTPUT); //porque de aqui saldra un pulso
  // Ponemos el pin Echo en modo entrada
  pinMode(PinEcho, INPUT); //porque recibira un valor

  /*****************Para los LED*******************************/
  pinMode(LED1, OUTPUT); //Establecemos el pin 14 (D5) como salida
  pinMode(LED2, OUTPUT); //Establecemos el pin 12 (D6) como salida

  /******Para el sensor DHT22 y el de humedad de suelo*********/
  dht.begin(); //Permite configurar el pin del sensor como INPUT_PULLUP
  pinMode(sensorHumedad, INPUT); //Definimos el pin del sensor como INPUT

  /************************************************************/
  Serial.begin(115200); //Se inicio la comunicación serial

  setup_wifi(); //Llamamos a la función para conectarnos a nuestra red wifi
  client.setServer(mqtt_server, 1883); //Enviamos el servidor y nuestro puerto
  client.setCallback(callback); //Establecemos el callback
}

/******************************************************************************/
void loop() {

  /*********************Para el sensor DHT22********************/
  //Se lee la temperatura en Celcius y se le asigna a la variable tem
  float tem = dht.readTemperature();
  //Se lee la humedad relativa y se le asigna a la variable hum
  float hum = dht.readHumidity();  //Se lee la humedad relativa y se le asigna a hum

  /***************Para el sensor de humedad del suelo************/
  // Capturamos el valor del sensor de humedad de suelo en la variable hum_suelo
  // Realizamos un pequeño calculo para transformar la lectura analogica a un rango de 0-100
  hum_suelo = (100.00 - ((analogRead(sensorHumedad) * 100) / 1024.00));

  /***************Para el sensor ultrasonido************/
  //configuramos el trigger
  digitalWrite(PinTrig, LOW); //Para generar un pulso limpio se pone a LOW por 4 us
  delayMicroseconds(4);
  digitalWrite(PinTrig, HIGH); //generamos un Trigger de 10 us
  delayMicroseconds(10);
  digitalWrite(PinTrig, LOW); //volvemos a poner en LOW

  //calculamos el tiempo en microsegundos (us) que le toma hacer contacto con un objeto
  tiempo = pulseIn(PinEcho, HIGH);

  //La distancia se calcula dividiento el tiempo entre 58.3
  //esto es porque se sabe que la velocidad del sonidos recorre
  //1cm en 29.15us, y como solo nos interesa la mitad de esa distancia
  //lo dividimos entre 2, por lo que con la regla de tres simple
  //tenemos que la distancia=tiempo/(29.15*2)
  distancia = tiempo / 58.3;
  lecturas[nActual] = distancia;
  nActual++;

  float promDistancia; //Valor promedio de 100 lecturas del sensor ultrasonido
  //Calculamos un promedio de las distancias para tratar de evitar cualquier tipo de error
  //en las lecturas del sensor ultrasonido
  if (nActual + 1 == 100) { //si ya hemos llenado el arreglo, entonces ...
    nActual = 0; //Reiniciamos el contador para volver a llenar el arreglo desde el indice 0
    promDistancia = promedio(lecturas); //Almacenamos el promedio de las distancias
    //Como lo que mostraremos en el Node-red seran el porcentaje de agua que hay en el recipiente
    //entonces aplicamos unas simples operaciones matematicas para obtenerlo
    porcentajeAgua = (1 - ((promDistancia - distanciaLleno) / (distanciaVacio - distanciaLleno))) * 100;
    //Agregamos una validaciones frente a posibles movimientos o perturbaciones en el sensor ultrasonido
    if (porcentajeAgua < 0) {
      porcentajeAgua = 0;
    } else if (porcentajeAgua > 100) {
      porcentajeAgua = 100;
    }
    //Para el encendido del led 2
    if (porcentajeAgua < 20) { //Si el porcentaje de agua del recipiente esta por debajo del 20%, entonces...
      digitalWrite(LED2, HIGH); //encendemos el led2
    } else {
      digitalWrite(LED2, LOW); //apagamos el led2
    }
  }
  delay(10); //esperamos 10 milisegundos

  /********************************************************************/
  //Si nos desconectamos trateremos de volver a conetarnos
  if (!client.connected()) {
    reconnect();//llamamos a la función reconnect
  }
  client.loop(); //hacemos un loop

  riego_automatico(tem, hum, hum_suelo); //Llamamos a la función del riego automatico

  long now = millis(); //Calculamos el tiempo actual
  //Si el tiempo actual menos el del ultimo mensaje es mayor igual a 5000
  //se publicaran los datos
  if (now - lastMsg >= 5000) {
    lastMsg = now; //actualizamos el momento en donde se recibe el ultimo mensaje

    //La funcion dtostrf nos permite convertir una variable de tipo double o float en su
    //representación ASCII y almacenarla en un arreglo de tipo char, que seran posteriormente publicadas,
    //primera se coloca la variable que deseamos convertir, luego el ancho de esta, el numero de decimales
    //y por ultimo donde queremos almacenarla
    dtostrf(tem, 0, 2, dht22tem);
    dtostrf(hum, 0, 2, dht22hum);
    dtostrf(hum_suelo, 0, 2, sensorhum);
    dtostrf(porcentajeAgua, 0, 2, sensorultrasonic);

    //Imprimimos los datos en el puerto serial
    Serial.print(tem);
    Serial.print(",");
    Serial.print(hum);
    Serial.print(",");
    Serial.print(hum_suelo);
    Serial.print(",");
    Serial.print(porcentajeAgua);
    Serial.print(",");
    Serial.print(estadoBomba);
    Serial.println();
    //Publicamos los datos
    client.publish("dht22-tem", dht22tem); //publicamos la temperatura
    client.publish("dht22-hum", dht22hum); //publicamos la humedad
    client.publish("sensor-hum_suelo", sensorhum); //publicamos la humedad del suelo
    client.publish("sensor-ultrasonido", sensorultrasonic); //publicamos el porcentaje de agua del recipiente

    delay(500); //esperamos 500 milisegundos
  }
}

/******************************************************************************/
void riego_automatico(float tem, float hum, float hum_suelo) {
  //Iniciamos con la bomba apagada
  digitalWrite(bombaAgua, HIGH); //apagar bomba de agua
  delay(100); //esperamos 100 milisegundos
  if ((tem > 30 && hum < 30 && hum_suelo < 30 ) || (hum_suelo < 20 )) {
    //el LED1 se encendera por 5 segundos antes de activarse el riego y estara encendida hasta que termine el riego
    digitalWrite(LED1, HIGH); //encendemos el led1
    delay(5000);
    digitalWrite(bombaAgua, LOW); //prender bomba de agua
    estadoBomba = "1"; //Actualizamos los estados
    delay(5000); //la bomba estara encendidad por 5 segundos
  }
  else if (mensaje == "control_riego") {
    if (inform == "1") {
      digitalWrite(bombaAgua, LOW); //prender bomba de agua
      digitalWrite(LED1, HIGH); //encendemos el led1 mientras la bomba este encendida
      //Actualizamos los estados
      estadoBomba = "1";
      delay(5000); //Activamos el riego por 5 segundos
    }
    else {
      digitalWrite(bombaAgua, HIGH); //apagar bomba de agua
      digitalWrite(LED1, LOW); //apagamos el led1
      //Actualizamos los estados
      estadoBomba = "0";
    }
  }
  else { //si no pasa nada de los anterior estara apagada
    digitalWrite(bombaAgua, HIGH); //apagar bomba de agua
    digitalWrite(LED1, LOW); //tambien apagamos el led1
    //Actualizamos los estados
    estadoBomba = "0";
  }
}

/******************************************************************************/
//Creamos una función para calcular el promedio de las lecturas en el arreglo
float promedio(float a[]) {
  float total = 0.0;
  for (int i = 0; i < nLecturas; i++) {
    total += a[i];
  }
  return total / nLecturas;
}
