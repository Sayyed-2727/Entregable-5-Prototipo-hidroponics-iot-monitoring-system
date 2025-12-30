# Memoria Descriptiva: Prototipo de Sistema de Riego Hidropónico IoT

**Autor:** Miguel Puch Paíno  
**Autor:** Sayed Magdy Elsayed Abdellah  
**Asignatura:** Sistemas Ciber-Físicos
**Máster:** Máster en Ingeniería Informática  
**Fecha:** Diciembre 2025

---

## Introducción

Este documento detalla el diseño, la arquitectura y la implementación de un prototipo funcional para un sistema de monitorización y control de un invernadero hidropónico. El objetivo principal es demostrar la viabilidad técnica de un sistema IoT completo, integrando hardware (sensores y actuadores), un microcontrolador avanzado, protocolos de comunicación estándar y una interfaz de usuario web para la visualización y control en tiempo real.

El sistema ha sido desarrollado siguiendo las especificaciones y diseños definidos en entregables anteriores, culminando en un prototipo que, aunque simulado, representa un modelo fiel y funcional de un producto final. La solución adoptada se adhiere a las mejores prácticas de desarrollo de software empotrado, incluyendo el uso de un sistema operativo de tiempo real (FreeRTOS), gestión eficiente de energía y una arquitectura de software modular y escalable.

---

## 1. Diseño Final del Nodo IoT

El nodo IoT es el corazón del sistema, encargado de la adquisición de datos del entorno y el control de los actuadores. Está basado en un microcontrolador **ESP32**, simulado en el entorno **Wokwi**.

### 1.1. Diagrama de Conexiones

El siguiente diagrama, generado en Wokwi, ilustra las conexiones físicas entre el microcontrolador ESP32 y los periféricos (sensores y actuadores).

*(Nota: Este diagrama es una representación conceptual del archivo `diagram.json` del proyecto.)*

![Diagrama de Wokwi](httpsd://i.imgur.com/example.png "Representación del circuito en Wokwi")

*Figura 1: Diagrama de conexiones del circuito en Wokwi.*

### 1.2. Tabla Descriptiva de Componentes

| Componente                 | Propósito                                     | Conexión ESP32        | Tipo de Señal |
| -------------------------- | --------------------------------------------- | --------------------- | ------------- |
| **ESP32**                  | Microcontrolador principal con WiFi y BT      | -                     | -             |
| **Sensor DHT22**           | Medición de temperatura y humedad ambiental   | `GPIO 15`             | Digital       |
| **Fotorresistencia (LDR)** | Medición de la intensidad lumínica            | `GPIO 34` (ADC1_CH6)  | Analógica     |
| **Potenciómetro Deslizante** | Simulación del nivel de agua en el depósito | `GPIO 35` (ADC1_CH7)  | Analógica     |
| **Potenciómetro de Control** | Ajuste dinámico del umbral de humedad        | `GPIO 33` (ADC1_CH5)  | Analógica     |
| **LED Azul**               | Indicador visual de la "Bomba de Riego"       | `GPIO 25`             | Digital (PWM) |
| **LED Verde**              | Indicador visual de la "Bomba de Nutrientes"  | `GPIO 26`             | Digital (PWM) |
| **Botón Pulsador**         | Cambio manual entre modos de operación        | `GPIO 32`             | Digital (IRQ) |
| **LCD 16x2 I2C**           | Pantalla para visualización de estado local   | `SDA:21`, `SCL:22`    | I2C           |

---

## 2. Arquitectura IoT Desarrollada

La arquitectura del sistema se ha diseñado para ser desacoplada, escalable y eficiente, utilizando el protocolo **MQTT** como eje central de la comunicación.

### 2.1. Diagrama de Arquitectura

El siguiente diagrama describe el flujo de información entre los componentes del sistema:

```mermaid
graph TD
    subgraph "Nodo IoT (Wokwi)"
        A(ESP32 con Sensores y Actuadores)
    end

    subgraph "Broker en la Nube"
        B(HiveMQ Public Broker)
    end

    subgraph "Cliente de Usuario"
        C(Dashboard Web<br>HTML, CSS, JS)
    end

    A --"1. Publica datos (TCP)"--> B
    B --"2. Reenvía datos (WebSocket)"--> C
    C --"3. Envía comandos (WebSocket)"--> B
    B --"4. Reenvía comandos (TCP)"--> A

    style B fill:#333,stroke:#fff,stroke-width:2px,color:#fff
```

*Figura 2: Diagrama de la arquitectura IoT y flujo de datos.*

### 2.2. Descripción de Componentes y Flujo

1.  **Nodo IoT (ESP32):** El firmware del ESP32, escrito en C++ sobre FreeRTOS, se conecta a la red WiFi "Wokwi-GUEST". Lee los datos de los sensores, evalúa la lógica de control según el modo activo y publica periódicamente mensajes JSON en diferentes **topics MQTT** (`invernadero/nodo1/sensores`, `invernadero/nodo1/actuadores`, `invernadero/nodo1/estado`) a través de una conexión TCP estándar con el broker. También se suscribe al topic `invernadero/nodo1/comando` para recibir órdenes desde el dashboard.

2.  **Broker MQTT (HiveMQ):** Se utiliza un broker MQTT público (`broker.hivemq.com`). Actúa como intermediario, recibiendo todos los mensajes publicados por el ESP32 y distribuyéndolos a cualquier cliente suscrito. Este desacoplamiento permite que el nodo IoT y la interfaz de usuario no necesiten conocerse directamente.

3.  **Dashboard Web (Cliente):** Es una aplicación de una sola página construida en HTML, CSS y JavaScript.
4.  **Dashboard (Node-Red):** El sistema incluye un dashboard desarrollado en Node-Red que se conecta al broker MQTT. Node-Red actúa como middleware que:
   - Se suscribe a los topics de sensores, actuadores y estado para recibir datos en tiempo real
   - Publica comandos en el topic `invernadero/nodo1/comando` cuando el usuario interactúa con los controles
   - Presenta una interfaz web gráfica en `http://localhost:1880/ui` con gauges, botones y visualizaciones
   - Permite cambiar entre los modos de operación (AUTO, ECO, MANUAL) de forma intuitiva

---

## 3. Implementación de Casos de Uso

Se han implementado tres modos de operación que constituyen los principales casos de uso del sistema.

### Caso de Uso 1: Riego Automático Inteligente (Modo AUTO)

-   **Descripción:** El sistema opera de forma autónoma para mantener las condiciones óptimas del cultivo.
-   **Lógica:** La lógica de control, ejecutada cada 500 ms, verifica las siguientes condiciones:
    1.  El nivel de agua debe ser superior al umbral crítico (20%).
    2.  La humedad ambiental debe ser inferior al umbral configurado (por defecto, 50%).
    3.  La intensidad de luz debe ser superior a un mínimo (20%) para evitar riegos nocturnos.
    4.  Debe haber transcurrido un intervalo mínimo desde el último riego (30 segundos).
-   **Resultado:** Si todas las condiciones se cumplen, el sistema activa las bombas de riego y nutrientes durante 10 segundos. El dashboard refleja el cambio de estado de las bombas y los sensores en tiempo real.

### Caso de Uso 2: Ahorro de Energía (Modo ECO)

-   **Descripción:** Un modo de operación que prioriza la eficiencia energética, ideal para sistemas alimentados por batería.
-   **Lógica:** Similar al modo AUTO, pero con umbrales más estrictos y ciclos más largos para reducir la actividad.
    1.  Umbral de humedad más bajo (< 45%).
    2.  Umbral de luz más alto (> 30%).
    3.  Intervalo de riego extendido a 60 segundos.
    4.  Duración del riego reducida a 5 segundos.
-   **Resultado:** El sistema activa las bombas con menor frecuencia y durante menos tiempo. El firmware está preparado para entrar en modo **Deep Sleep** entre ciclos, guardando el estado del sistema en la memoria RTC del ESP32 para reanudar la operación correctamente tras despertar. (Nota: La función de Deep Sleep está comentada en el código final para permitir el monitoreo continuo desde el dashboard en la simulación).

### Caso de Uso 3: Control Manual Directo (Modo MANUAL)

-   **Descripción:** Permite al usuario tomar el control total sobre los actuadores para tareas de mantenimiento o pruebas.
-   **Lógica:**
    1.  El usuario selecciona "MANUAL" en el dashboard o mediante el botón físico en el nodo IoT.
    2.  La lógica de control automático se deshabilita.
    3.  Los interruptores de control de las bombas en el dashboard se habilitan.
-   **Resultado:** El usuario puede encender o apagar las bombas de riego y nutrientes a voluntad a través de la interfaz web. El sistema responde instantáneamente a los comandos enviados vía MQTT.

---