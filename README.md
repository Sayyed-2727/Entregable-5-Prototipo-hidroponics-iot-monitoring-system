# 🌱 Sistema IoT de Invernadero Hidropónico

Sistema completo de monitoreo y control automático para invernadero hidropónico basado en ESP32, con simulación en Wokwi, comunicación MQTT y dashboard web en tiempo real.

**Autor:** Miguel Puch Paíno  
**Fecha:** Diciembre 2025

---

## 📋 Índice

- [Descripción](#-descripción)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Hardware y Sensores](#️-hardware-y-sensores)
- [Modos de Operación](#-modos-de-operación)
- [Lógica de Activación de Bombas](#-lógica-de-activación-de-bombas)
- [Instalación y Ejecución](#-instalación-y-ejecución)
- [Interacción con el Sistema](#-interacción-con-el-sistema)
- [Estructura del Proyecto](#-estructura-del-proyecto)

---

## 🎯 Descripción

Sistema IoT completo que simula el control automático de un invernadero hidropónico. Monitorea temperatura, humedad, luz y nivel de agua, y controla automáticamente dos bombas (riego y nutrientes) según el modo de operación seleccionado.

**Características principales:**
- ✅ 3 modos de operación: AUTO, ECO y MANUAL
- ✅ Monitoreo en tiempo real de 4 sensores
- ✅ Control automático de 2 bombas
- ✅ Comunicación MQTT bidireccional
- ✅ Dashboard web responsive
- ✅ Simulación completa en Wokwi

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────┐      MQTT       ┌──────────────────┐
│   ESP32 + IoT   │◄───────────────►│  Broker MQTT     │
│   (Wokwi Web)   │  broker.hivemq  │  (HiveMQ Public) │
│                 │     :8000        └──────────────────┘
│  - DHT22        │                           ▲
│  - LDR          │                           │
│  - Potenciómetros│                          │
│  - 2 LEDs       │                           │
│  - LCD          │                           │
└─────────────────┘                           │
                                              │ WebSocket
                                              │ ws://...8000/mqtt
                                              │
                                   ┌──────────┴───────────┐
                                   │   Dashboard Web      │
                                   │   (localhost:8000)   │
                                   │                      │
                                   │  - Gráficos sensores │
                                   │  - Control de modos  │
                                   │  - Registro eventos  │
                                   └──────────────────────┘
```

---

## 🛠️ Hardware y Sensores

### Componentes

| Componente | Propósito | Pin ESP32 |
|------------|-----------|-----------|
| **DHT22** | Temperatura y humedad ambiente | GPIO 15 |
| **LDR** (Fotorresistencia) | Intensidad de luz | GPIO 34 (ADC) |
| **Potenciómetro deslizante** | Simulación nivel de agua | GPIO 35 (ADC) |
| **Potenciómetro circular** | Ajuste de parámetros | GPIO 33 (ADC) |
| **LED Azul** | Indicador bomba de riego | GPIO 25 |
| **LED Verde** | Indicador bomba de nutrientes | GPIO 26 |
| **Botón** | Cambio de modo (AUTO/ECO/MANUAL) | GPIO 32 |
| **LCD 16x2 I2C** | Display local (actualmente deshabilitado) | SDA:21, SCL:22 |

### Rangos de Valores

- **Temperatura:** -40°C a 80°C (DHT22)
- **Humedad:** 0% a 100% (DHT22)
- **Luz:** 0% a 100% (calculado desde LDR)
- **Nivel de Agua:** 0% a 100% (potenciómetro)

---

## 🎮 Modos de Operación

### 1️⃣ Modo AUTO

**Descripción:** Control automático inteligente basado en sensores.

**Características:**
- ✅ Monitoreo continuo cada 2 segundos
- ✅ Decisiones automáticas basadas en umbrales
- ✅ Considera luz, humedad y nivel de agua
- ✅ Intervalos de riego: cada 30 segundos
- ✅ Duración del riego: 10 segundos

**Condiciones para activar bombas:**
```
SI humedad < umbralHumedadMin (50%)
  Y luz > umbralLuzMin (20%)
  Y nivelAgua > nivelAguaCritico (20%)
  Y han pasado >= 30 segundos desde último riego
ENTONCES:
  → Activar bomba de riego (LED azul)
  → Activar bomba de nutrientes (LED verde)
  → Mantener activas durante 10 segundos
```

**Ejemplo práctico:**
- Humedad actual: **40%** → **Sí cumple** (< 50%)
- Luz actual: **60%** → **Sí cumple** (> 20%)
- Nivel agua: **70%** → **Sí cumple** (> 20%)
- Tiempo desde último riego: **35 seg** → **Sí cumple** (> 30s)
- **RESULTADO: BOMBAS SE ACTIVAN** ✅

### 2️⃣ Modo ECO

**Descripción:** Modo económico con menor consumo de recursos.

**Características:**
- ✅ Umbrales más estrictos (humedad < 45%)
- ✅ Intervalos más largos: cada 60 segundos
- ✅ Duración reducida: 5 segundos
- ✅ Requiere más luz (> 30%)

**Condiciones para activar bombas:**
```
SI humedad < (umbralHumedadMin - 5%) → 45%
  Y luz > (umbralLuzMin + 10%) → 30%
  Y nivelAgua > nivelAguaCritico (20%)
  Y han pasado >= 60 segundos desde último riego
ENTONCES:
  → Activar bombas durante 5 segundos
```

**Ejemplo práctico:**
- Humedad actual: **42%** → **Sí cumple** (< 45%)
- Luz actual: **50%** → **Sí cumple** (> 30%)
- Nivel agua: **65%** → **Sí cumple** (> 20%)
- Tiempo desde último riego: **70 seg** → **Sí cumple** (> 60s)
- **RESULTADO: BOMBAS SE ACTIVAN** ✅

### 3️⃣ Modo MANUAL

**Descripción:** Control directo por el usuario.

**Características:**
- ✅ Los switches del dashboard se habilitan
- ✅ Usuario controla bombas directamente
- ✅ No hay lógica automática
- ✅ Las bombas permanecen como las configure el usuario

**Controles disponibles:**
- 🔵 **Switch Bomba de Riego:** ON/OFF directo
- 🟢 **Switch Bomba de Nutrientes:** ON/OFF directo

**Ejemplo de uso:**
1. Cambiar a modo MANUAL desde dashboard
2. Activar switch "Bomba de Riego" → LED azul se enciende
3. Activar switch "Bomba de Nutrientes" → LED verde se enciende
4. Las bombas permanecen activas hasta que las desactives manualmente

---

## 💧 Lógica de Activación de Bombas

### Tabla de Condiciones

| Condición | Modo AUTO | Modo ECO | Modo MANUAL |
|-----------|-----------|----------|-------------|
| **Umbral de humedad** | < 50% | < 45% | N/A |
| **Umbral de luz** | > 20% | > 30% | N/A |
| **Nivel de agua mínimo** | > 20% | > 20% | > 20% |
| **Intervalo de riego** | 30 segundos | 60 segundos | N/A |
| **Duración del riego** | 10 segundos | 5 segundos | Ilimitado |
| **Control** | Automático | Automático | Manual |

### ¿Por qué NO se activan las bombas?

**Escenario 1: Nivel de agua crítico**
```
❌ Nivel de agua: 15% (< 20%)
→ Las bombas NO se activarán en ningún modo
→ Solución: Aumentar potenciómetro de nivel de agua
```

**Escenario 2: Humedad alta**
```
❌ Humedad: 65% (> 50% en AUTO)
→ No hay necesidad de riego
→ Solución: Disminuir humedad en sensor DHT22
```

**Escenario 3: Luz insuficiente (solo AUTO/ECO)**
```
❌ Luz: 15% (< 20% en AUTO)
→ Sistema no riega de noche
→ Solución: Aumentar luz en sensor LDR
```

**Escenario 4: Intervalo no cumplido**
```
❌ Último riego hace: 15 segundos (< 30s en AUTO)
→ Esperando intervalo de seguridad
→ Solución: Esperar a que se cumpla el intervalo
```

### Indicadores Visuales

| Estado | LED Azul | LED Verde | Dashboard |
|--------|----------|-----------|-----------|
| **Inactivo** | 🔵 Apagado | 🟢 Apagado | "Inactiva" (gris) |
| **Regando** | 🔵 Encendido | 🟢 Encendido | "Activa" (verde) |
| **Agua crítica** | 🔵 Apagado | 🟢 Apagado | ⚠️ "NIVEL CRÍTICO" |

---

## 🚀 Instalación y Ejecución

### Requisitos Previos

- ✅ Navegador web moderno (Chrome, Edge, Firefox)
- ✅ Python 3.x instalado
- ✅ Cuenta en Wokwi.com (gratuita)

### Paso 1: Ejecutar Simulación en Wokwi Web

1. **Ir a Wokwi:** https://wokwi.com/projects/new/esp32

2. **Copiar sketch.ino:**
   - Seleccionar TODO el contenido de `sketch.ino` local
   - Pegar en Wokwi (pestaña sketch.ino)
   - Guardar (Ctrl+S)

3. **Copiar diagram.json:**
   - Hacer click en pestaña "diagram.json"
   - Borrar contenido existente
   - Pegar el contenido de `diagram.json` local
   - Guardar (Ctrl+S)

4. **Copiar libraries.txt:**
   - Click en botón "+" → "Add File"
   - Nombrar: `libraries.txt`
   - Pegar contenido de `libraries.txt` local
   - Guardar

5. **Iniciar simulación:**
   - Click en botón verde **Play** ▶️
   - Esperar a que compile y cargue bibliotecas
   - Verificar en Serial Monitor:
     ```
     === Sistema IoT Invernadero Hidropónico ===
     Pines configurados
     DHT22 inicializado
     Conectando a WiFi...
     WiFi conectado
     Conectado a MQTT
     T:24.00°C H:45.00% Luz:24% Agua:6%
     Datos publicados en MQTT
     ```

### Paso 2: Iniciar Dashboard Web

1. **Abrir terminal en la carpeta del proyecto**

2. **Iniciar servidor HTTP:**
   ```bash
   python -m http.server 8000
   ```

3. **Abrir navegador:**
   - Ir a: http://localhost:8000
   - Debería aparecer el dashboard
   - Verificar: "MQTT Conectado" (verde)

4. **Verificar datos:**
   - Los valores de sensores deben aparecer automáticamente
   - Revisar "Registro de Eventos" para mensajes

---

## 🎯 Interacción con el Sistema

### Ajustar Sensores en Wokwi

1. **DHT22 (Temperatura/Humedad):**
   - Click en el sensor DHT22
   - Arrastrar slider "Temperature": 15°C a 35°C
   - Arrastrar slider "Humidity": 20% a 80%
   - **Prueba:** Poner humedad a **40%** para activar riego

2. **LDR (Luz):**
   - Click en el fotorresistor
   - Cambiar "Lux": 0 a 1000
   - **Prueba:** Poner > 200 lux (~40% de luz)

3. **Potenciómetro Nivel de Agua:**
   - Click en el potenciómetro deslizante
   - Mover slider izquierda/derecha
   - **Prueba:** Mantener > 50% para operación normal

4. **Potenciómetro de Control:**
   - Click en potenciómetro circular
   - Girar haciendo click y arrastrando
   - Ajusta umbral de humedad en tiempo real

5. **Botón de Modo:**
   - Click en botón rojo
   - Cambia entre AUTO → ECO → MANUAL → AUTO

### Controlar desde Dashboard

1. **Cambiar Modo:**
   - Click en botón "AUTO", "ECO" o "MANUAL"
   - El modo se actualiza en ESP32 y dashboard

2. **Control Manual (solo en modo MANUAL):**
   - Cambiar a modo MANUAL
   - Activar switch "Bomba de Riego"
   - Activar switch "Bomba de Nutrientes"
   - Observar LEDs en Wokwi

3. **Monitorear:**
   - Ver valores de sensores actualizándose
   - Revisar estado de bombas
   - Leer registro de eventos

### Escenarios de Prueba

**Prueba 1: Riego Automático**
```
1. Modo: AUTO
2. Ajustar DHT22: Humedad = 40%
3. Ajustar LDR: Luz = 60%
4. Nivel agua: > 50%
5. Esperar 30 segundos
6. RESULTADO: LEDs azul y verde se encienden
```

**Prueba 2: Modo ECO**
```
1. Cambiar a modo ECO
2. Ajustar DHT22: Humedad = 42%
3. Ajustar LDR: Luz = 50%
4. Esperar 60 segundos
5. RESULTADO: Riego de 5 segundos
```

**Prueba 3: Control Manual**
```
1. Cambiar a modo MANUAL
2. En dashboard: Activar switch "Bomba de Riego"
3. RESULTADO: LED azul se enciende inmediatamente
4. Desactivar switch
5. RESULTADO: LED azul se apaga
```

**Prueba 4: Nivel de Agua Crítico**
```
1. Modo: AUTO
2. Nivel agua: < 20% (mover potenciómetro izquierda)
3. RESULTADO: Bombas NO se activan
4. Dashboard muestra: "⚠️ NIVEL CRÍTICO"
```

---

## 📁 Estructura del Proyecto

```
Entregable-5-Prototipo-hidroponics-iot-monitoring-system/
│
├── sketch.ino              # Código ESP32 (firmware)
├── diagram.json            # Circuito Wokwi (componentes y conexiones)
├── libraries.txt           # Dependencias Arduino
├── wokwi.toml             # Configuración Wokwi
│
├── index.html             # Dashboard web (estructura)
├── style.css              # Estilos dashboard
├── script.js              # Lógica MQTT y frontend
│
└── README.md              # Esta documentación
```

---
