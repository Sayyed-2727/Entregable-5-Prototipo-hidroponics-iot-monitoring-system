/*
 * Sistema IoT de Invernadero Hidropónico - ESP32
 * Control automático de riego y dosificación de nutrientes
 * con WiFi, MQTT y pantalla LCD
 * 
 * Autor 1: Miguel Puch Paíno
 * Autor 2: Sayed Magdy Elsayed Abdellah
 * Fecha: Diciembre 2025
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// ==================== CONFIGURACIÓN WIFI Y MQTT ====================
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWORD = "";

// Topics MQTT
const char* TOPIC_SENSORES = "invernadero/nodo1/sensores";
const char* TOPIC_ACTUADORES = "invernadero/nodo1/actuadores";
const char* TOPIC_ESTADO = "invernadero/nodo1/estado";
const char* TOPIC_COMANDO = "invernadero/nodo1/comando";

// ==================== PINES DE SENSORES ====================
#define DHT_PIN 15
#define DHT_TYPE DHT22
#define LDR_PIN 34           // Entrada analógica para sensor de luz
#define NIVEL_POT_PIN 35     // Potenciómetro de nivel de agua

// ==================== PINES DE ACTUADORES ====================
#define BOMBA_RIEGO_PIN 25   // Relé/LED bomba de riego
#define BOMBA_NUTRIENTES_PIN 26 // Relé/LED bomba de nutrientes

// ==================== PINES DE CONTROLES ====================
#define BOTON_MODO_PIN 32    // Botón para cambiar modo
#define POT_CONTROL_PIN 33   // Potenciómetro de control

// ==================== OBJETOS ====================
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ==================== MODOS DE OPERACIÓN ====================
enum Modo { AUTO, ECO, MANUAL };
Modo modoActual = AUTO;
const char* modoNombres[] = {"AUTO", "ECO", "MANUAL"};

// ==================== VARIABLES DE SENSORES ====================
float temperatura = 0;
float humedad = 0;
int luzPorcentaje = 0;
int nivelAguaPorcentaje = 0;

// ==================== VARIABLES DE ACTUADORES ====================
bool bombaRiegoActiva = false;
bool bombaNutrientesActiva = false;

// ==================== PARÁMETROS CONFIGURABLES ====================
int umbralHumedadMin = 50;       // Humedad mínima para activar riego (aumentado para pruebas)
int umbralHumedadMax = 70;       // Humedad máxima
int umbralLuzMin = 20;           // Luz mínima para riego diurno (reducido)
int nivelAguaCritico = 20;       // Nivel crítico de agua (%)
unsigned long intervaloRiego = 30000;     // 30 segundos en modo AUTO (reducido para pruebas)
unsigned long intervaloRiegoEco = 60000;  // 1 minuto en modo ECO
unsigned long duracionRiego = 10000;      // 10 segundos
unsigned long duracionRiegoEco = 5000;    // 5 segundos en ECO

// ==================== CONTROL DE TIEMPO ====================
unsigned long ultimoRiego = 0;
unsigned long tiempoInicioRiego = 0;
unsigned long ultimaLecturaSensores = 0;
unsigned long ultimoEnvioMQTT = 0;
unsigned long ultimaActualizacionLCD = 0;
unsigned long ultimaVerificacionBoton = 0;

const unsigned long INTERVALO_LECTURA = 2000;
const unsigned long INTERVALO_MQTT = 5000;
const unsigned long INTERVALO_LCD = 2000;
const unsigned long DEBOUNCE_BOTON = 200;

// ==================== CONTROL DE PANTALLA ====================
int pantallaActual = 0;
const int NUM_PANTALLAS = 3;

// ==================== ESTADO ANTERIOR DEL BOTÓN ====================
bool botonAnterior = HIGH;

// ==================== VARIABLES PARA RIEGO MANUAL ====================
bool riegoManualActivo = false;
bool nutrientesManualActivo = false;

// ==================== FUNCIONES DE CONFIGURACIÓN ====================

void setup() {
  Serial.begin(115200);
  delay(1000); // Esperar a que Serial esté listo
  Serial.println("\n\n=== Sistema IoT Invernadero Hidropónico ===");
  
  // Configurar pines
  configurarPines();
  Serial.println("Pines configurados");
  
  // Inicializar periféricos
  dht.begin();
  Serial.println("DHT22 inicializado");
  
  // Inicializar LCD con manejo de errores
  Wire.begin(21, 22);
  delay(100);
  lcd.init();
  lcd.backlight();
  Serial.println("LCD inicializado");
  
  mostrarMensajeBienvenida();
  
  Serial.println("Iniciando sistema...");
  
  // Conectar WiFi
  conectarWiFi();
  
  // Configurar MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  
  Serial.println("Sistema inicializado correctamente");
}

void configurarPines() {
  // Actuadores
  pinMode(BOMBA_RIEGO_PIN, OUTPUT);
  pinMode(BOMBA_NUTRIENTES_PIN, OUTPUT);
  digitalWrite(BOMBA_RIEGO_PIN, LOW);
  digitalWrite(BOMBA_NUTRIENTES_PIN, LOW);
  
  // Controles
  pinMode(BOTON_MODO_PIN, INPUT_PULLUP);
}

void mostrarMensajeBienvenida() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Invernadero IoT");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...");
  delay(2000);
}

void conectarWiFi() {
  Serial.print("Conectando a WiFi");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando WiFi");
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(intentos % 16, 1);
    lcd.print(".");
    intentos++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi OK");
    lcd.setCursor(0, 1);
    // lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("\nError al conectar WiFi");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi ERROR");
    delay(2000);
  }
}

void conectarMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("Conectando a MQTT...");
    
    String clientId = "ESP32_Nodo1_" + String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("MQTT conectado");
      mqttClient.subscribe(TOPIC_COMANDO);
      
      // Publicar mensaje de conexión
      StaticJsonDocument<200> doc;
      doc["nodo"] = "nodo1";
      doc["estado"] = "online";
      doc["timestamp"] = millis();
      
      char buffer[200];
      serializeJson(doc, buffer);
      mqttClient.publish(TOPIC_ESTADO, buffer);
    } else {
      Serial.print("Error MQTT, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String mensaje = "";
  for (int i = 0; i < length; i++) {
    mensaje += (char)payload[i];
  }
  Serial.println(mensaje);
  
  // Procesar comandos
  if (String(topic) == TOPIC_COMANDO) {
    procesarComando(mensaje);
  }
}

void procesarComando(String comando) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, comando);
  
  if (error) {
    Serial.print("Error al parsear JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Cambiar modo
  if (doc.containsKey("modo")) {
    String modo = doc["modo"];
    if (modo == "AUTO") modoActual = AUTO;
    else if (modo == "ECO") modoActual = ECO;
    else if (modo == "MANUAL") modoActual = MANUAL;
    Serial.print("Modo cambiado a: ");
    Serial.println(modoNombres[modoActual]);
  }
  
  // Control manual
  if (modoActual == MANUAL) {
    if (doc.containsKey("riego")) {
      riegoManualActivo = doc["riego"];
    }
    if (doc.containsKey("nutrientes")) {
      nutrientesManualActivo = doc["nutrientes"];
    }
  }
  
  // Ajustar umbrales
  if (doc.containsKey("umbral_humedad")) {
    umbralHumedadMin = doc["umbral_humedad"];
  }
}

// ==================== BUCLE PRINCIPAL ====================

void loop() {
  unsigned long tiempoActual = millis();
  
  // Mantener conexión MQTT
  if (!mqttClient.connected()) {
    conectarMQTT();
  }
  mqttClient.loop();
  
  // Leer sensores
  if (tiempoActual - ultimaLecturaSensores >= INTERVALO_LECTURA) {
    leerSensores();
    ultimaLecturaSensores = tiempoActual;
  }
  
  // Verificar botón de modo
  if (tiempoActual - ultimaVerificacionBoton >= DEBOUNCE_BOTON) {
    verificarBotonModo();
    ultimaVerificacionBoton = tiempoActual;
  }
  
  // Leer potenciómetro de control
  leerPotenciometroControl();
  
  // Ejecutar lógica según modo
  switch (modoActual) {
    case AUTO:
      ejecutarModoAuto(tiempoActual);
      break;
    case ECO:
      ejecutarModoEco(tiempoActual);
      break;
    case MANUAL:
      ejecutarModoManual();
      break;
  }
  
  // Actualizar actuadores
  actualizarBombas();
  
  // Actualizar LCD
  if (tiempoActual - ultimaActualizacionLCD >= INTERVALO_LCD) {
    actualizarLCD();
    ultimaActualizacionLCD = tiempoActual;
  }
  
  // Publicar datos MQTT
  if (tiempoActual - ultimoEnvioMQTT >= INTERVALO_MQTT) {
    publicarDatosMQTT();
    ultimoEnvioMQTT = tiempoActual;
  }
  
  delay(10);
}

// ==================== FUNCIONES DE SENSORES ====================

void leerSensores() {
  // Leer DHT22
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  
  if (!isnan(t) && !isnan(h)) {
    temperatura = t;
    humedad = h;
  }
  
  // Leer sensor de luz (LDR)
  int valorLDR = analogRead(LDR_PIN);
  luzPorcentaje = map(valorLDR, 0, 4095, 0, 100);
  
  // Leer nivel de agua (usando potenciómetro)
  int valorNivel = analogRead(NIVEL_POT_PIN);
  nivelAguaPorcentaje = map(valorNivel, 0, 4095, 0, 100);
  
  // Debug
  Serial.print("T:");
  Serial.print(temperatura);
  Serial.print("°C H:");
  Serial.print(humedad);
  Serial.print("% Luz:");
  Serial.print(luzPorcentaje);
  Serial.print("% Agua:");
  Serial.print(nivelAguaPorcentaje);
  Serial.println("%");
}

void leerPotenciometroControl() {
  int valorPot = analogRead(POT_CONTROL_PIN);
  // Ajustar umbral de humedad entre 20 y 80%
  umbralHumedadMin = map(valorPot, 0, 4095, 20, 80);
}

void verificarBotonModo() {
  bool estadoBoton = digitalRead(BOTON_MODO_PIN);
  
  if (estadoBoton == LOW && botonAnterior == HIGH) {
    // Botón presionado
    modoActual = (Modo)((modoActual + 1) % 3);
    Serial.print("Modo cambiado a: ");
    Serial.println(modoNombres[modoActual]);
    
    // Reiniciar variables de modo manual
    if (modoActual != MANUAL) {
      riegoManualActivo = false;
      nutrientesManualActivo = false;
    }
    
    // Mostrar cambio en LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Modo: ");
    lcd.print(modoNombres[modoActual]);
    delay(1000);
  }
  
  botonAnterior = estadoBoton;
}

// ==================== LÓGICA DE CONTROL ====================

void ejecutarModoAuto(unsigned long tiempoActual) {
  // Verificar si hay nivel de agua suficiente
  if (nivelAguaPorcentaje < nivelAguaCritico) {
    bombaRiegoActiva = false;
    bombaNutrientesActiva = false;
    return;
  }
  
  // Control de riego activo
  if (bombaRiegoActiva) {
    if (tiempoActual - tiempoInicioRiego >= duracionRiego) {
      // Terminar riego
      bombaRiegoActiva = false;
      bombaNutrientesActiva = false;
      ultimoRiego = tiempoActual;
    }
    return;
  }
  
  // Verificar si debe iniciar riego
  bool debeRegar = false;
  
  // Condición 1: Humedad baja
  if (humedad < umbralHumedadMin) {
    debeRegar = true;
  }
  
  // Condición 2: Luz suficiente (periodo diurno)
  if (luzPorcentaje < umbralLuzMin) {
    debeRegar = false;  // No regar de noche
  }
  
  // Condición 3: Intervalo de tiempo
  if (tiempoActual - ultimoRiego < intervaloRiego) {
    debeRegar = false;
  }
  
  if (debeRegar) {
    iniciarRiego(tiempoActual);
  }
}

void ejecutarModoEco(unsigned long tiempoActual) {
  // Similar a AUTO pero con intervalos más largos y duración menor
  if (nivelAguaPorcentaje < nivelAguaCritico) {
    bombaRiegoActiva = false;
    bombaNutrientesActiva = false;
    return;
  }
  
  if (bombaRiegoActiva) {
    if (tiempoActual - tiempoInicioRiego >= duracionRiegoEco) {
      bombaRiegoActiva = false;
      bombaNutrientesActiva = false;
      ultimoRiego = tiempoActual;
    }
    return;
  }
  
  bool debeRegar = false;
  
  // Umbrales más estrictos en modo ECO
  if (humedad < (umbralHumedadMin - 5)) {
    debeRegar = true;
  }
  
  if (luzPorcentaje < (umbralLuzMin + 10)) {
    debeRegar = false;
  }
  
  if (tiempoActual - ultimoRiego < intervaloRiegoEco) {
    debeRegar = false;
  }
  
  if (debeRegar) {
    iniciarRiego(tiempoActual);
  }
}

void ejecutarModoManual() {
  // En modo manual, control directo
  if (nivelAguaPorcentaje < nivelAguaCritico) {
    riegoManualActivo = false;
    nutrientesManualActivo = false;
  }
  
  bombaRiegoActiva = riegoManualActivo;
  bombaNutrientesActiva = nutrientesManualActivo;
}

void iniciarRiego(unsigned long tiempoActual) {
  bombaRiegoActiva = true;
  bombaNutrientesActiva = true;  // Dosificar nutrientes durante riego
  tiempoInicioRiego = tiempoActual;
  
  Serial.println("=== RIEGO INICIADO ===");
}

void actualizarBombas() {
  digitalWrite(BOMBA_RIEGO_PIN, bombaRiegoActiva ? HIGH : LOW);
  digitalWrite(BOMBA_NUTRIENTES_PIN, bombaNutrientesActiva ? HIGH : LOW);
}

// ==================== INTERFAZ LCD ====================

void actualizarLCD() {
  lcd.clear();
  
  switch (pantallaActual) {
    case 0:
      mostrarPantallaSensores();
      break;
    case 1:
      mostrarPantallaActuadores();
      break;
    case 2:
      mostrarPantallaEstado();
      break;
  }
  
  // Rotar pantallas
  pantallaActual = (pantallaActual + 1) % NUM_PANTALLAS;
}

void mostrarPantallaSensores() {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  // lcd.print((int)temperatura);
  lcd.print("C H:");
  // lcd.print((int)humedad);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(luzPorcentaje);
  lcd.print("% W:");
  lcd.print(nivelAguaPorcentaje);
  lcd.print("%");
  
  // Indicador visual de nivel de agua
  lcd.setCursor(15, 1);
  if (nivelAguaPorcentaje > 70) {
    // lcd.print((char)255);  // Bloque lleno
  } else if (nivelAguaPorcentaje > 40) {
    lcd.print("~");
  } else {
    lcd.print("!");
  }
}

void mostrarPantallaActuadores() {
  lcd.setCursor(0, 0);
  lcd.print("BOMBAS:");
  
  lcd.setCursor(0, 1);
  lcd.print("R:");
  lcd.print(bombaRiegoActiva ? "ON " : "OFF");
  lcd.print(" N:");
  lcd.print(bombaNutrientesActiva ? "ON " : "OFF");
  
  // Símbolos de cultivo
  lcd.setCursor(13, 0);
  if (bombaRiegoActiva) {
    // lcd.print((char)126); // ~
  }
  
  lcd.setCursor(14, 0);
  if (luzPorcentaje > 50) {
    lcd.print("*");  // Sol
  }
  
  lcd.setCursor(15, 0);
  if (humedad > 60) {
    // lcd.print((char)255);  // Planta creciendo
  } else {
    lcd.print("v");
  }
}

void mostrarPantallaEstado() {
  lcd.setCursor(0, 0);
  lcd.print("Modo:");
  lcd.print(modoNombres[modoActual]);
  
  lcd.setCursor(0, 1);
  lcd.print("Umbral:");
  lcd.print(umbralHumedadMin);
  lcd.print("%");
  
  // Estado de conexión
  lcd.setCursor(13, 1);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("W");
  }
  if (mqttClient.connected()) {
    lcd.print("M");
  }
}

// ==================== COMUNICACIÓN MQTT ====================

void publicarDatosMQTT() {
  if (!mqttClient.connected()) {
    return;
  }
  
  // Publicar sensores
  StaticJsonDocument<300> docSensores;
  docSensores["temperatura"] = temperatura;
  docSensores["humedad"] = humedad;
  docSensores["luz"] = luzPorcentaje;
  docSensores["nivel_agua"] = nivelAguaPorcentaje;
  docSensores["timestamp"] = millis();
  
  char bufferSensores[300];
  serializeJson(docSensores, bufferSensores);
  mqttClient.publish(TOPIC_SENSORES, bufferSensores);
  
  // Publicar actuadores
  StaticJsonDocument<200> docActuadores;
  docActuadores["bomba_riego"] = bombaRiegoActiva;
  docActuadores["bomba_nutrientes"] = bombaNutrientesActiva;
  docActuadores["timestamp"] = millis();
  
  char bufferActuadores[200];
  serializeJson(docActuadores, bufferActuadores);
  mqttClient.publish(TOPIC_ACTUADORES, bufferActuadores);
  
  // Publicar estado
  StaticJsonDocument<200> docEstado;
  docEstado["modo"] = modoNombres[modoActual];
  docEstado["umbral_humedad"] = umbralHumedadMin;
  docEstado["wifi_conectado"] = (WiFi.status() == WL_CONNECTED);
  docEstado["mqtt_conectado"] = mqttClient.connected();
  docEstado["timestamp"] = millis();
  
  char bufferEstado[200];
  serializeJson(docEstado, bufferEstado);
  mqttClient.publish(TOPIC_ESTADO, bufferEstado);
  
  Serial.println("Datos publicados en MQTT");
}


