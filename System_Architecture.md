# System Architecture Diagram

```mermaid
graph TD
    subgraph "Sistema Hidropónico"
        subgraph "Sensores"
            direction LR
            DHT22(DHT22<br>Temp/Humedad) -->|Pin 15| ESP32
            LDR(LDR<br>Sensor de Luz) -->|Pin 34| ESP32
            WaterLevel(Potenciómetro<br>Nivel de Agua) -->|Pin 35| ESP32
        end

        subgraph "Actuadores"
            direction RL
            ESP32 -->|Pin 25| WaterPump(Bomba de Riego)
            ESP32 -->|Pin 26| NutrientPump(Bomba de Nutrientes)
        end

        subgraph "Controles de Usuario"
            direction LR
            ModeButton(Botón de Modo) -->|Pin 32| ESP32
            ControlPot(Potenciómetro<br>Control Umbral) -->|Pin 33| ESP32
        end

        subgraph "Interfaz Local"
            direction RL
            ESP32 -->|I2C (0x27)| LCD(LCD 16x2)
        end

        ESP32(ESP32<br>Microcontrolador)
    end

    subgraph "Comunicación IoT"
        ESP32 -- "WiFi" --> Router(Router WiFi<br>Wokwi-GUEST)
        Router -- "Internet" --> MQTTBroker(MQTT Broker<br>broker.hivemq.com)
        MQTTBroker <--> RemoteClient(Cliente Remoto<br>Dashboard/App)
    end

    subgraph "Flujo de Datos MQTT"
        direction TB
        ESP32 -- "Publica" --> TopicSensores(Topic: /sensores)
        ESP32 -- "Publica" --> TopicActuadores(Topic: /actuadores)
        ESP32 -- "Publica" --> TopicEstado(Topic: /estado)
        TopicComando(Topic: /comando) -- "Subscrito" --> ESP32

        TopicSensores --> MQTTBroker
        TopicActuadores --> MQTTBroker
        TopicEstado --> MQTTBroker
        MQTTBroker --> TopicComando
    end
```
