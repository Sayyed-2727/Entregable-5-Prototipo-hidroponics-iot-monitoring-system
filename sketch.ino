/*
 * Sistema IoT de Invernadero Hidropónico - ESP32 con FreeRTOS
 * Control automático de riego y dosificación de nutrientes
 * con WiFi, MQTT y pantalla LCD.
 *
 * Arquitectura refactorizada con FreeRTOS y drivers ESP-IDF para una
 * gestión de tareas, precisión y consumo energético optimizados.
 *
 * Autor 1: Miguel Puch Paíno
 * Autor 2: Sayed Magdy Elsayed Abdellah
 * Fecha: Diciembre 2025
 */

// Librerías de alto nivel (Arduino)
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// Headers ESP-IDF para control de bajo nivel
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"


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

// ==================== PINES Y CANALES ====================
// Sensores
#define DHT_PIN 15
#define DHT_TYPE DHT22
#define LDR_PIN 34
#define NIVEL_POT_PIN 35

// ADC (Mapeo de pines a canales de ADC1)
const adc1_channel_t LDR_ADC_CHANNEL = ADC1_CHANNEL_6; // GPIO34
const adc1_channel_t NIVEL_POT_ADC_CHANNEL = ADC1_CHANNEL_7; // GPIO35
const adc1_channel_t POT_CONTROL_ADC_CHANNEL = ADC1_CHANNEL_5; // GPIO33

// Actuadores (usando el enum gpio_num_t para compatibilidad con ESP-IDF)
#define BOMBA_RIEGO_PIN GPIO_NUM_25
#define BOMBA_NUTRIENTES_PIN GPIO_NUM_26

// Controles (usando el enum gpio_num_t)
#define BOTON_MODO_PIN GPIO_NUM_32
#define POT_CONTROL_PIN 33

// PWM (LEDC)
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_RIEGO_CANAL LEDC_CHANNEL_0
#define PWM_NUTRIENTES_CANAL LEDC_CHANNEL_1
#define VELOCIDAD_MAX 255
#define VELOCIDAD_ECO 180

// ==================== OBJETOS Y HANDLES ====================
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Handles de Tareas y Sincronización
TaskHandle_t xTaskControlLogicHandle = NULL;
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xButtonSemaphore;
EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// ADC
esp_adc_cal_characteristics_t adc_chars;
const adc_atten_t atten = ADC_ATTEN_DB_11;
const adc_bits_width_t width = ADC_WIDTH_BIT_12;

// ==================== ESTADO DEL SISTEMA Y DATOS COMPARTIDOS ====================

// --- Modos de Operación ---
enum Modo { AUTO, ECO, MANUAL };
const char* modoNombres[] = {"AUTO", "ECO", "MANUAL"};

// --- Estado RTC (persiste durante deep sleep) ---
struct rtc_state_t {
  Modo modoActual = AUTO;
  unsigned long ultimoRiego = 0;
};
RTC_DATA_ATTR rtc_state_t rtcState;

// --- Variables de estado en RAM (sincronizadas desde RTC) ---
Modo modoActual = AUTO;
unsigned long ultimoRiego = 0;

// --- Variables de Sensores (protegidas por mutex) ---
float temperatura = 0;
float humedad = 0;
int luzPorcentaje = 0;
int nivelAguaPorcentaje = 0;

// --- Variables de Actuadores (protegidas por mutex) ---
bool bombaRiegoActiva = false;
bool bombaNutrientesActiva = false;

// --- Parámetros Configurables (protegidos por mutex) ---
int umbralHumedadMin = 50;
int umbralHumedadMax = 70;
int umbralLuzMin = 20;
int nivelAguaCritico = 20;
unsigned long intervaloRiego = 30000;
// Intervalo para despertar de deep sleep en modo ECO (en microsegundos)
uint64_t intervaloRiegoEco = 60000000; 
unsigned long duracionRiego = 10000;
unsigned long duracionRiegoEco = 5000;

// --- Control de Tiempo (usado por la lógica de control) ---
unsigned long tiempoInicioRiego = 0;

// --- Riego Manual ---
bool riegoManualActivo = false;
bool nutrientesManualActivo = false;

// ==================== DECLARACIÓN DE TAREAS Y FUNCIONES ====================

// --- Tareas FreeRTOS ---
void taskMqttHandler(void *pvParameters);
void taskReadSensors(void *pvParameters);
void taskReadControls(void *pvParameters);
void taskButtonHandler(void *pvParameters);
void taskControlLogic(void *pvParameters);
void taskUpdateLcd(void *pvParameters);

// --- Funciones de Configuración (ESP-IDF) ---
void configurarADC();
void configurarControlBombas();
void configurarTimerControl();
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void configurarWiFiEficiente();

// --- Funciones de Lógica y Control ---
void restaurarEstadoRTC();
void entrarModoBajoConsumo();
void mostrarMensajeBienvenida();
void conectarWiFi();
void conectarMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void procesarComando(String comando);
uint32_t leerADCCalibrado(adc1_channel_t channel);
void leerSensores();
void leerPotenciometroControl();
void ejecutarModoAuto(unsigned long tiempoActual);
void ejecutarModoEco(unsigned long tiempoActual);
void ejecutarModoManual();
void iniciarRiego(unsigned long tiempoActual);
void actualizarBombas();
void actualizarLCD();
void publicarDatosMQTT();

// --- Rutinas de Servicio de Interrupción (ISRs) ---
static void IRAM_ATTR handleButtonInterrupt(void* arg);
bool IRAM_ATTR timerControlISR(void *args);


// ==================== FUNCIÓN DE CONFIGURACIÓN PRINCIPAL ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Sistema IoT Invernadero Hidropónico con ESP-IDF ===");

  // Comprobar la causa del reinicio
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Despertando de Deep Sleep (Timer).");
    restaurarEstadoRTC();
  } else {
    Serial.println("Inicio en frío (Power on).");
    // No es necesario hacer nada, rtcState tiene los valores por defecto
  }

  // Sincronizar estado de RAM con RTC al inicio
  modoActual = rtcState.modoActual;
  ultimoRiego = rtcState.ultimoRiego;

  // --- Inicialización de periféricos ESP-IDF ---
  configurarADC();
  Serial.println("ADC configurado para alta precisión.");
  configurarControlBombas();
  Serial.println("PWM (LEDC) configurado para control de bombas.");
  
  // --- Inicialización de periféricos Arduino ---
  dht.begin();
  Serial.println("DHT22 inicializado.");
  Wire.begin(21, 22);
  delay(100);
  lcd.init();
  lcd.backlight();
  Serial.println("LCD inicializado.");
  
  // Conectar WiFi y MQTT solo si no estamos en modo ECO al despertar
  if (modoActual != ECO || wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) {
      mostrarMensajeBienvenida();
      conectarWiFi();
  } else {
      Serial.println("Omitiendo conexión WiFi/MQTT en despertar ECO.");
  }


  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // --- Creación de Mutex y Semáforos ---
  xDataMutex = xSemaphoreCreateMutex();
  xButtonSemaphore = xSemaphoreCreateBinary();

  // --- Configuración de Interrupciones (ESP-IDF) ---
  gpio_install_isr_service(0);
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << BOTON_MODO_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE
  };
  gpio_config(&io_conf);
  gpio_isr_handler_add(BOTON_MODO_PIN, handleButtonInterrupt, NULL);
  Serial.println("Interrupción de botón (ESP-IDF) configurada.");

  // Iniciar timer de hardware para la lógica de control
  configurarTimerControl();
  Serial.println("Timer de hardware para control lógico iniciado.");

  // --- Creación de Tareas FreeRTOS ---
  Serial.println("Creando tareas de FreeRTOS...");
  xTaskCreatePinnedToCore(taskMqttHandler, "MqttHandler", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskReadSensors, "ReadSensors", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskReadControls, "ReadControls", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskButtonHandler, "ButtonHandler", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskUpdateLcd, "UpdateLcd", 4096, NULL, 1, NULL, 1);
  // Crear la tarea de control y guardar su handle para la notificación del timer
  xTaskCreatePinnedToCore(taskControlLogic, "ControlLogic", 4096, NULL, 2, &xTaskControlLogicHandle, 1);
  
  Serial.println("Sistema inicializado. El planificador de FreeRTOS tiene el control.");
  vTaskDelete(NULL); // Eliminar la tarea de setup/loop
}

// ==================== BUCLE PRINCIPAL (VACÍO) ====================
void loop() {
  // No hacer nada aquí. El planificador de FreeRTOS se encarga de todo.
}

// ==================== RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISRs) ====================

// ISR para el botón de cambio de modo (API ESP-IDF)
static void IRAM_ATTR handleButtonInterrupt(void* arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// ISR para el timer de hardware que notifica a la tarea de control
bool IRAM_ATTR timerControlISR(void *args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Notificar a la tarea de control que es hora de ejecutarse
    vTaskNotifyGiveFromISR(xTaskControlLogicHandle, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}


// ==================== IMPLEMENTACIÓN DE TAREAS FREERTOS ====================

void taskMqttHandler(void *pvParameters) {
  TickType_t lastPublishTime = 0;
  const TickType_t publishInterval = pdMS_TO_TICKS(5000);

  for (;;) {
    // Esperar a que la WiFi esté conectada
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    
    if (!mqttClient.connected()) {
      conectarMQTT();
    }
    mqttClient.loop();

    if (xTaskGetTickCount() - lastPublishTime >= publishInterval) {
      if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        publicarDatosMQTT();
        xSemaphoreGive(xDataMutex);
      }
      lastPublishTime = xTaskGetTickCount();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void taskReadSensors(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      leerSensores();
      xSemaphoreGive(xDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskReadControls(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      leerPotenciometroControl();
      xSemaphoreGive(xDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void taskButtonHandler(void *pvParameters) {
  for (;;) {
    // 1. Esperar a que la interrupción señale una posible pulsación del botón.
    if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdTRUE) {
      
      // 2. Retardo para antirrebote (debounce).
      vTaskDelay(pdMS_TO_TICKS(50));

      // 3. Confirmar que el botón sigue presionado después del retardo.
      if (gpio_get_level(BOTON_MODO_PIN) == 0) {
        
        Serial.println("Interrupt: Pulsación de botón confirmada.");
        
        // 4. Actualizar el modo de forma segura usando el mutex.
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
          modoActual = (Modo)((modoActual + 1) % 3);
          Serial.print("Modo cambiado a: "); Serial.println(modoNombres[modoActual]);
          
          if (modoActual != MANUAL) {
            riegoManualActivo = false;
            nutrientesManualActivo = false;
          }
          
          xSemaphoreGive(xDataMutex);
        }

        // 5. Esperar a que el botón se suelte para registrarlo como una única pulsación.
        while (gpio_get_level(BOTON_MODO_PIN) == 0) {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
      }
      
      // 6. Después de gestionar la pulsación y liberación, limpiar cualquier 
      //    interrupción remanente de rebotes que pudieran haber ocurrido.
      while(xSemaphoreTake(xButtonSemaphore, 0) == pdTRUE);
    }
  }
}

void taskControlLogic(void *pvParameters) {
  for (;;) {
    // Esperar notificación del timer de hardware en lugar de usar vTaskDelay
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    unsigned long tiempoActual = millis();
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      switch (modoActual) {
        case AUTO:   ejecutarModoAuto(tiempoActual);   break;
        case ECO:    ejecutarModoEco(tiempoActual);    break;
        case MANUAL: ejecutarModoManual();             break;
      }
      actualizarBombas();
      xSemaphoreGive(xDataMutex);
    }
  }
}

void taskUpdateLcd(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      actualizarLCD();
      xSemaphoreGive(xDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ==================== CONFIGURACIÓN DE PERIFÉRICOS (ESP-IDF) ====================

void configurarADC() {
  adc1_config_width(width);
  adc1_config_channel_atten(LDR_ADC_CHANNEL, atten);
  adc1_config_channel_atten(NIVEL_POT_ADC_CHANNEL, atten);
  adc1_config_channel_atten(POT_CONTROL_ADC_CHANNEL, atten);
  esp_adc_cal_characterize(ADC_UNIT_1, atten, width, 1100, &adc_chars);
}

void configurarControlBombas() {
  // Configurar timer para LEDC
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // Configurar canal para bomba de riego
  ledc_channel_config_t channel_conf_riego = {
      .gpio_num = BOMBA_RIEGO_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = PWM_RIEGO_CANAL,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
  };
  ledc_channel_config(&channel_conf_riego);

  // Configurar canal para bomba de nutrientes
  ledc_channel_config_t channel_conf_nutrientes = {
      .gpio_num = BOMBA_NUTRIENTES_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = PWM_NUTRIENTES_CANAL,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
  };
  ledc_channel_config(&channel_conf_nutrientes);
}

void configurarTimerControl() {
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80 // 1 MHz (80 MHz / 80)
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 500000); // Notificar cada 0.5 segundos
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timerControlISR, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        Serial.println("WiFi desconectado. Reintentando...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        Serial.print("WiFi conectado. IP: ");
        // Corregir el tipo de puntero para ip4addr_ntoa
        Serial.println(ip4addr_ntoa((const ip4_addr_t*)&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void configurarWiFiEficiente() {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            // Se establece el modo de autenticación a OPEN para redes sin contraseña
            .threshold = {.authmode = WIFI_AUTH_OPEN},
            .pmf_cfg = {.capable = true, .required = false},
        },
    };
    // Copiar SSID y password usando strcpy como requiere la API de ESP-IDF
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM)); // Ahorro de energía
}


// ==================== FUNCIONES DE LÓGICA Y CONTROL ====================

void restaurarEstadoRTC() {
  // Esta función se llama si despertamos de deep sleep.
  // El estado ya está en la variable rtcState.
  // Lo que hacemos es copiarlo a las variables de trabajo en RAM.
  modoActual = rtcState.modoActual;
  ultimoRiego = rtcState.ultimoRiego;
  Serial.printf("Estado restaurado desde RTC: Modo=%s, UltimoRiego=%lu\n", modoNombres[modoActual], ultimoRiego);
}

void entrarModoBajoConsumo() {
    Serial.println("Modo ECO: Preparando para entrar en Deep Sleep.");
    
    // Sincronizar el estado actual a la variable RTC antes de dormir
    rtcState.modoActual = modoActual;
    rtcState.ultimoRiego = ultimoRiego;

    Serial.printf("Guardando estado en RTC: Modo=%s, UltimoRiego=%lu\n", modoNombres[rtcState.modoActual], rtcState.ultimoRiego);
    
    // Configurar el timer como la fuente para despertar
    esp_sleep_enable_timer_wakeup(intervaloRiegoEco);
    
    // Entrar en modo de sueño profundo
    esp_deep_sleep_start();
}


void mostrarMensajeBienvenida() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Invernadero IoT");
  lcd.setCursor(0, 1); lcd.print("Iniciando...");
  delay(2000);
}

void conectarWiFi() {
  Serial.print("Conectando a WiFi");
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Conectando WiFi");
  
  configurarWiFiEficiente();
  
  // Esperar a que el bit de conexión se active en el event group
  EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(20000)); // 20 seg timeout

  if (bits & WIFI_CONNECTED_BIT) {
    Serial.println("\nWiFi conectado (evento recibido)");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi OK");
  } else {
    Serial.println("\nError al conectar WiFi (timeout)");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi ERROR");
  }
  delay(2000);
}

void conectarMQTT() {
  Serial.print("Conectando a MQTT...");
  String clientId = "ESP32_Nodo1_" + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    Serial.println("MQTT conectado");
    mqttClient.subscribe(TOPIC_COMANDO);
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      publicarDatosMQTT();
      xSemaphoreGive(xDataMutex);
    }
  } else {
    Serial.print("Error MQTT, rc="); Serial.println(mqttClient.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String mensaje = "";
  for (int i = 0; i < length; i++) { mensaje += (char)payload[i]; }
  Serial.print("Mensaje recibido ["); Serial.print(topic); Serial.print("]: "); Serial.println(mensaje);
  if (String(topic) == TOPIC_COMANDO) {
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      procesarComando(mensaje);
      xSemaphoreGive(xDataMutex);
    }
  }
}

void procesarComando(String comando) {
  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, comando).code() != DeserializationError::Ok) {
    Serial.println("Error al parsear JSON de comando"); return;
  }
  if (doc.containsKey("modo")) {
    String modo = doc["modo"];
    if (modo == "AUTO") modoActual = AUTO;
    else if (modo == "ECO") modoActual = ECO;
    else if (modo == "MANUAL") modoActual = MANUAL;
    Serial.print("Modo cambiado vía MQTT a: "); Serial.println(modoNombres[modoActual]);
  }
  if (modoActual == MANUAL) {
    if (doc.containsKey("riego")) riegoManualActivo = doc["riego"];
    if (doc.containsKey("nutrientes")) nutrientesManualActivo = doc["nutrientes"];
  }
  if (doc.containsKey("umbral_humedad")) umbralHumedadMin = doc["umbral_humedad"];
}

uint32_t leerADCCalibrado(adc1_channel_t channel) {
    uint32_t raw_reading = 0;
    // Promediar múltiples lecturas para reducir ruido
    for (int i = 0; i < 16; i++) {
        raw_reading += adc1_get_raw(channel);
    }
    raw_reading /= 16;
    // Convertir lectura raw a milivoltios
    return esp_adc_cal_raw_to_voltage(raw_reading, &adc_chars);
}

void leerSensores() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h)) { temperatura = t; humedad = h; }

  uint32_t ldr_mV = leerADCCalibrado(LDR_ADC_CHANNEL);
  uint32_t nivel_mV = leerADCCalibrado(NIVEL_POT_ADC_CHANNEL);

  luzPorcentaje = map(ldr_mV, 0, 3100, 0, 100); // Rango de 0 a 3.1V con atenuación de 11dB
  nivelAguaPorcentaje = map(nivel_mV, 0, 3100, 0, 100);

  Serial.printf("Sensores -> T:%.1fC | H:%.1f%% | Luz:%d%% (%dmV) | Agua:%d%% (%dmV)\n", temperatura, humedad, luzPorcentaje, ldr_mV, nivelAguaPorcentaje, nivel_mV);
}

void leerPotenciometroControl() {
  uint32_t control_mV = leerADCCalibrado(POT_CONTROL_ADC_CHANNEL);
  umbralHumedadMin = map(control_mV, 0, 3100, 20, 80);
}

void ejecutarModoAuto(unsigned long tiempoActual) {
  if (nivelAguaPorcentaje < nivelAguaCritico) {
    bombaRiegoActiva = false; bombaNutrientesActiva = false; return;
  }
  if (bombaRiegoActiva) {
    if (tiempoActual - tiempoInicioRiego >= duracionRiego) {
      bombaRiegoActiva = false; bombaNutrientesActiva = false;
      ultimoRiego = tiempoActual;
    }
    return;
  }
  bool debeRegar = (humedad < umbralHumedadMin) && (luzPorcentaje >= umbralLuzMin) && (tiempoActual - ultimoRiego >= intervaloRiego);
  if (debeRegar) {
    iniciarRiego(tiempoActual);
  }
}

void ejecutarModoEco(unsigned long tiempoActual) {
  bool debeRegar = (humedad < (umbralHumedadMin - 5)) && (luzPorcentaje >= (umbralLuzMin + 10));
  if (debeRegar) {
      // En lugar de regar directamente, iniciamos la secuencia de riego y luego dormimos
      iniciarRiego(tiempoActual);
      // La lógica de apagado de la bomba se ejecutará en la siguiente iteración de la tarea de control
      // antes de ir a dormir.
  } else if (!bombaRiegoActiva) {
      // Si no hay que regar y las bombas no están activas, entramos en bajo consumo.
      entrarModoBajoConsumo();
  }

  // Lógica para apagar la bomba si estaba encendida
  if (bombaRiegoActiva) {
    if (tiempoActual - tiempoInicioRiego >= duracionRiegoEco) {
      bombaRiegoActiva = false; bombaNutrientesActiva = false;
      ultimoRiego = tiempoActual;
    }
  }
}

void ejecutarModoManual() {
  if (nivelAguaPorcentaje < nivelAguaCritico) {
    riegoManualActivo = false; nutrientesManualActivo = false;
  }
  bombaRiegoActiva = riegoManualActivo;
  bombaNutrientesActiva = nutrientesManualActivo;
}

void iniciarRiego(unsigned long tiempoActual) {
  bombaRiegoActiva = true;
  bombaNutrientesActiva = true;
  tiempoInicioRiego = tiempoActual;
  Serial.println("=== RIEGO INICIADO ===");
}

void actualizarBombas() {
  uint32_t duty_riego = bombaRiegoActiva ? VELOCIDAD_MAX : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_RIEGO_CANAL, duty_riego);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_RIEGO_CANAL);

  uint32_t duty_nutrientes = bombaNutrientesActiva ? VELOCIDAD_MAX : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_NUTRIENTES_CANAL, duty_nutrientes);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_NUTRIENTES_CANAL);
}

int pantallaActual = 0;
const int NUM_PANTALLAS = 3;

void actualizarLCD() {
  lcd.clear();
  float temp_lcd; int hum_lcd, luz_lcd, nivel_lcd, umbral_lcd;
  bool riego_lcd, nutr_lcd;
  Modo modo_lcd;

  temp_lcd = temperatura; hum_lcd = (int)humedad; luz_lcd = luzPorcentaje;
  nivel_lcd = nivelAguaPorcentaje; umbral_lcd = umbralHumedadMin;
  riego_lcd = bombaRiegoActiva; nutr_lcd = bombaNutrientesActiva;
  modo_lcd = modoActual;

  switch (pantallaActual) {
    case 0:
      lcd.setCursor(0, 0); lcd.print("T:" + String((int)temp_lcd) + "C H:" + String(hum_lcd) + "%");
      lcd.setCursor(0, 1); lcd.print("L:" + String(luz_lcd) + "% W:" + String(nivel_lcd) + "%");
      break;
    case 1:
      lcd.setCursor(0, 0); lcd.print("BOMBAS:");
      lcd.setCursor(0, 1); lcd.print("R:" + String(riego_lcd ? "ON " : "OFF") + " N:" + String(nutr_lcd ? "ON " : "OFF"));
      break;
    case 2:
      lcd.setCursor(0, 0); lcd.print("Modo:" + String(modoNombres[modo_lcd]));
      lcd.setCursor(0, 1); lcd.print("Umbral:" + String(umbral_lcd) + "%");
      lcd.setCursor(13, 1);
      if (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) lcd.print("W");
      if (mqttClient.connected()) lcd.print("M");
      break;
  }
  pantallaActual = (pantallaActual + 1) % NUM_PANTALLAS;
}

void publicarDatosMQTT() {
  if (!mqttClient.connected()) return;
  char buffer[300];
  StaticJsonDocument<300> docSensores;
  docSensores["temperatura"] = temperatura;
  docSensores["humedad"] = humedad;
  docSensores["luz"] = luzPorcentaje;
  docSensores["nivel_agua"] = nivelAguaPorcentaje;
  serializeJson(docSensores, buffer);
  mqttClient.publish(TOPIC_SENSORES, buffer);

  StaticJsonDocument<200> docActuadores;
  docActuadores["bomba_riego"] = bombaRiegoActiva;
  docActuadores["bomba_nutrientes"] = bombaNutrientesActiva;
  serializeJson(docActuadores, buffer);
  mqttClient.publish(TOPIC_ACTUADORES, buffer);

  StaticJsonDocument<200> docEstado;
  docEstado["modo"] = modoNombres[modoActual];
  docEstado["umbral_humedad"] = umbralHumedadMin;
  docEstado["wifi_conectado"] = (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT);
  serializeJson(docEstado, buffer);
  mqttClient.publish(TOPIC_ESTADO, buffer);

  Serial.println("Datos publicados en MQTT");
}