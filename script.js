/**
 * =========================================================================
 * DASHBOARD IOT - INVERNADERO HIDROPÓNICO
 * =========================================================================
 * 
 * Sistema web de monitoreo y control en tiempo real para invernadero
 * hidropónico basado en ESP32, comunicación MQTT y visualización web.
 * 
 * @author Miguel Puch Paíno
 * @author Sayed Magdy Elsayed Abdellah 
 * @date Diciembre 2025
 * 
 * Funcionalidades principales:
 * - Conexión MQTT bidireccional (WebSocket)
 * - Monitoreo de 4 sensores en tiempo real
 * - Control de 2 actuadores (bombas)
 * - 3 modos de operación (AUTO, ECO, MANUAL)
 * - Registro de eventos con timestamps
 * =========================================================================
 */

// ==================== CONFIGURACIÓN MQTT ====================
/**
 * Configuración del broker MQTT y parámetros de conexión.
 * 
 * IMPORTANTE: ESP32 usa puerto 1883 (TCP), Frontend usa puerto 8000 (WebSocket)
 */
const MQTT_CONFIG = {
    broker: 'ws://broker.hivemq.com:8000/mqtt',
    clientId: 'dashboard_' + Math.random().toString(16).substr(2, 8),
    options: {
        keepalive: 60,
        clean: true,
        reconnectPeriod: 5000,
    }
};

/**
 * Topics MQTT utilizados para comunicación con el ESP32.
 */
// Topics MQTT (coherentes con el sketch.ino)
const TOPICS = {
    sensores: 'invernadero/nodo1/sensores',      // ESP32 publica datos de sensores
    actuadores: 'invernadero/nodo1/actuadores',  // ESP32 publica estado de bombas
    estado: 'invernadero/nodo1/estado',          // ESP32 publica estado general
    comando: 'invernadero/nodo1/comando'         // Frontend publica comandos al ESP32
};

// ==================== VARIABLES GLOBALES ====================
let mqttClient = null;
let modoActual = 'AUTO';
let ultimaActualizacion = null;

// ==================== INICIALIZACIÓN ====================
document.addEventListener('DOMContentLoaded', function() {
    console.log('Dashboard IoT inicializado');
    
    // Configurar cliente ID en el modal
    document.getElementById('mqtt-client-id').value = MQTT_CONFIG.clientId;
    
    // Ocultar modal (conectar automáticamente)
    ocultarModalMQTT();
    
    // Configurar event listeners
    configurarEventListeners();
    
    // Conectar automáticamente al cargar
    setTimeout(() => {
        conectarMQTT();
        agregarLog('Esperando datos del ESP32 en Wokwi...', 'info');
    }, 500);
});

// ==================== CONFIGURACIÓN DE EVENTOS ====================
function configurarEventListeners() {
    // Botón de conexión MQTT
    document.getElementById('connect-btn').addEventListener('click', function() {
        const broker = document.getElementById('mqtt-broker').value;
        MQTT_CONFIG.broker = broker;
        conectarMQTT();
        ocultarModalMQTT();
    });
    
    // Botones de modo
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.addEventListener('click', function() {
            const modo = this.getAttribute('data-mode');
            cambiarModo(modo);
        });
    });
    
    // Switches de control manual
    document.getElementById('switch-riego').addEventListener('change', function() {
        if (modoActual === 'MANUAL') {
            enviarComandoManual('riego', this.checked);
        }
    });
    
    document.getElementById('switch-nutrientes').addEventListener('change', function() {
        if (modoActual === 'MANUAL') {
            enviarComandoManual('nutrientes', this.checked);
        }
    });
}

// ==================== CONEXIÓN MQTT ====================
function conectarMQTT() {
    agregarLog('Conectando a broker MQTT...', 'info');
    
    try {
        mqttClient = mqtt.connect(MQTT_CONFIG.broker, MQTT_CONFIG.options);
        
        mqttClient.on('connect', function() {
            console.log('Conectado a MQTT');
            actualizarEstadoConexion(true);
            agregarLog('Conexión MQTT establecida correctamente', 'success');
            
            // Suscribirse a los topics
            mqttClient.subscribe([
                TOPICS.sensores,
                TOPICS.actuadores,
                TOPICS.estado
            ], function(err) {
                if (!err) {
                    agregarLog('Suscrito a topics del invernadero', 'success');
                } else {
                    agregarLog('Error al suscribirse: ' + err, 'error');
                }
            });
        });
        
        mqttClient.on('message', function(topic, message) {
            procesarMensajeMQTT(topic, message.toString());
        });
        
        mqttClient.on('error', function(err) {
            console.error('Error MQTT:', err);
            agregarLog('Error de conexión MQTT', 'error');
            actualizarEstadoConexion(false);
        });
        
        mqttClient.on('close', function() {
            console.log('Conexión MQTT cerrada');
            actualizarEstadoConexion(false);
            agregarLog('Conexión MQTT cerrada', 'warning');
        });
        
        mqttClient.on('reconnect', function() {
            console.log('Reconectando a MQTT...');
            agregarLog('Intentando reconectar...', 'info');
        });
        
    } catch (error) {
        console.error('Error al conectar MQTT:', error);
        agregarLog('Error al inicializar MQTT: ' + error.message, 'error');
        actualizarEstadoConexion(false);
    }
}

// ==================== PROCESAMIENTO DE MENSAJES ====================
function procesarMensajeMQTT(topic, mensaje) {
    try {
        const datos = JSON.parse(mensaje);
        console.log('Mensaje recibido:', topic, datos);
        
        switch(topic) {
            case TOPICS.sensores:
                actualizarSensores(datos);
                break;
            case TOPICS.actuadores:
                actualizarActuadores(datos);
                break;
            case TOPICS.estado:
                actualizarEstado(datos);
                break;
        }
        
        // Actualizar timestamp
        ultimaActualizacion = new Date();
        actualizarTimestamp();
        
    } catch (error) {
        console.error('Error al procesar mensaje:', error);
        agregarLog('Error al procesar mensaje JSON', 'error');
    }
}

// ==================== ACTUALIZACIÓN DE SENSORES ====================
function actualizarSensores(datos) {
    // Temperatura
    if (datos.temperatura !== undefined) {
        const temp = parseFloat(datos.temperatura).toFixed(1);
        document.getElementById('temperatura').textContent = temp;
        actualizarBarra('temperatura-bar', temp, 0, 50);
    }
    
    // Humedad
    if (datos.humedad !== undefined) {
        const hum = parseFloat(datos.humedad).toFixed(1);
        document.getElementById('humedad').textContent = hum;
        actualizarBarra('humedad-bar', hum, 0, 100);
    }
    
    // Luz
    if (datos.luz !== undefined) {
        const luz = parseInt(datos.luz);
        document.getElementById('luz').textContent = luz;
        actualizarBarra('luz-bar', luz, 0, 100);
    }
    
    // Nivel de agua
    if (datos.nivel_agua !== undefined) {
        const nivel = parseInt(datos.nivel_agua);
        document.getElementById('nivel-agua').textContent = nivel;
        actualizarBarra('nivel-agua-bar', nivel, 0, 100);
        
        // Mostrar alerta si es necesario
        const alertDiv = document.getElementById('agua-alert');
        if (nivel < 20) {
            alertDiv.textContent = '⚠️ NIVEL CRÍTICO';
            alertDiv.className = 'card-alert show critical';
        } else if (nivel < 40) {
            alertDiv.textContent = '⚠️ Nivel Bajo';
            alertDiv.className = 'card-alert show warning';
        } else {
            alertDiv.className = 'card-alert';
        }
    }
    
    agregarLog('Sensores actualizados', 'info');
}

// ==================== ACTUALIZACIÓN DE ACTUADORES ====================
function actualizarActuadores(datos) {
    // Bomba de riego
    if (datos.bomba_riego !== undefined) {
        const activa = datos.bomba_riego;
        const indicator = document.getElementById('bomba-riego-indicator');
        const status = document.getElementById('bomba-riego-status');
        
        if (activa) {
            indicator.classList.add('active');
            status.textContent = 'Activa';
            status.classList.add('active');
            agregarLog('💧 Bomba de riego activada', 'success');
        } else {
            indicator.classList.remove('active');
            status.textContent = 'Inactiva';
            status.classList.remove('active');
        }
    }
    
    // Bomba de nutrientes
    if (datos.bomba_nutrientes !== undefined) {
        const activa = datos.bomba_nutrientes;
        const indicator = document.getElementById('bomba-nutrientes-indicator');
        const status = document.getElementById('bomba-nutrientes-status');
        
        if (activa) {
            indicator.classList.add('active');
            status.textContent = 'Activa';
            status.classList.add('active');
            agregarLog('🧪 Bomba de nutrientes activada', 'success');
        } else {
            indicator.classList.remove('active');
            status.textContent = 'Inactiva';
            status.classList.remove('active');
        }
    }
}

// ==================== ACTUALIZACIÓN DE ESTADO ====================
function actualizarEstado(datos) {
    // Modo de operación
    if (datos.modo !== undefined) {
        modoActual = datos.modo;
        actualizarModoUI(modoActual);
    }
    
    // Umbral de humedad
    if (datos.umbral_humedad !== undefined) {
        const umbral = parseInt(datos.umbral_humedad);
        document.getElementById('umbral-humedad').textContent = umbral;
        actualizarBarra('umbral-humedad-bar', umbral, 20, 80);
    }
    
    // Estado WiFi
    if (datos.wifi_conectado !== undefined) {
        const wifiStatus = document.getElementById('wifi-status');
        if (datos.wifi_conectado) {
            wifiStatus.textContent = '🟢 Conectado';
            wifiStatus.style.color = 'var(--success)';
        } else {
            wifiStatus.textContent = '🔴 Desconectado';
            wifiStatus.style.color = 'var(--danger)';
        }
    }
    
    agregarLog('Estado del sistema actualizado', 'info');
}

// ==================== ACTUALIZAR MODO UI ====================
function actualizarModoUI(modo) {
    // Actualizar badge
    const badge = document.getElementById('modo-badge');
    badge.textContent = modo;
    badge.className = 'mode-badge ' + modo;
    
    // Actualizar descripción
    const descriptions = {
        'AUTO': 'Control automático basado en sensores',
        'ECO': 'Modo económico con reducción de consumo',
        'MANUAL': 'Control manual directo de actuadores'
    };
    document.getElementById('modo-description').textContent = descriptions[modo] || '';
    
    // Actualizar botones
    document.querySelectorAll('.mode-btn').forEach(btn => {
        if (btn.getAttribute('data-mode') === modo) {
            btn.classList.add('active');
        } else {
            btn.classList.remove('active');
        }
    });
    
    // Habilitar/deshabilitar controles manuales
    const switchRiego = document.getElementById('switch-riego');
    const switchNutrientes = document.getElementById('switch-nutrientes');
    
    if (modo === 'MANUAL') {
        switchRiego.disabled = false;
        switchNutrientes.disabled = false;
        agregarLog('Modo MANUAL: Controles habilitados', 'warning');
    } else {
        switchRiego.disabled = true;
        switchNutrientes.disabled = true;
        switchRiego.checked = false;
        switchNutrientes.checked = false;
    }
}

// ==================== COMANDOS DE CONTROL ====================
function cambiarModo(modo) {
    if (!mqttClient || !mqttClient.connected) {
        agregarLog('Error: No hay conexión MQTT', 'error');
        return;
    }
    
    const comando = {
        modo: modo
    };
    
    const mensaje = JSON.stringify(comando);
    mqttClient.publish(TOPICS.comando, mensaje, function(err) {
        if (!err) {
            agregarLog(`Cambio a modo ${modo} enviado`, 'success');
            modoActual = modo;
            actualizarModoUI(modo);
        } else {
            agregarLog('Error al enviar comando', 'error');
        }
    });
}

function enviarComandoManual(tipo, estado) {
    if (!mqttClient || !mqttClient.connected) {
        agregarLog('Error: No hay conexión MQTT', 'error');
        return;
    }
    
    const comando = {
        modo: 'MANUAL'
    };
    
    if (tipo === 'riego') {
        comando.riego = estado;
    } else if (tipo === 'nutrientes') {
        comando.nutrientes = estado;
    }
    
    const mensaje = JSON.stringify(comando);
    mqttClient.publish(TOPICS.comando, mensaje, function(err) {
        if (!err) {
            agregarLog(`Control manual: ${tipo} ${estado ? 'ON' : 'OFF'}`, 'success');
        } else {
            agregarLog('Error al enviar comando manual', 'error');
        }
    });
}

function aplicarUmbral(umbral) {
    if (!mqttClient || !mqttClient.connected) {
        agregarLog('Error: No hay conexión MQTT', 'error');
        return;
    }
    
    const comando = {
        umbral_humedad: umbral
    };
    
    const mensaje = JSON.stringify(comando);
    mqttClient.publish(TOPICS.comando, mensaje, function(err) {
        if (!err) {
            agregarLog(`Umbral de humedad ajustado a ${umbral}%`, 'success');
        } else {
            agregarLog('Error al ajustar umbral', 'error');
        }
    });
}

// ==================== UTILIDADES UI ====================
function actualizarBarra(elementId, valor, min, max) {
    const porcentaje = ((valor - min) / (max - min)) * 100;
    const elemento = document.getElementById(elementId);
    if (elemento) {
        elemento.style.width = Math.min(100, Math.max(0, porcentaje)) + '%';
    }
}

function actualizarEstadoConexion(conectado) {
    const statusBadge = document.getElementById('mqtt-status');
    if (conectado) {
        statusBadge.className = 'status-badge connected';
        statusBadge.innerHTML = '<span class="status-dot"></span>MQTT Conectado';
    } else {
        statusBadge.className = 'status-badge disconnected';
        statusBadge.innerHTML = '<span class="status-dot"></span>MQTT Desconectado';
    }
}

function actualizarTimestamp() {
    if (ultimaActualizacion) {
        const tiempo = ultimaActualizacion.toLocaleTimeString('es-ES');
        document.getElementById('ultima-actualizacion').textContent = tiempo;
    }
}

// ==================== REGISTRO DE EVENTOS ====================
function agregarLog(mensaje, tipo = 'info') {
    const logContainer = document.getElementById('log-container');
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry log-${tipo}`;
    
    const ahora = new Date();
    const tiempo = ahora.toLocaleTimeString('es-ES');
    
    logEntry.innerHTML = `
        <span class="log-time">${tiempo}</span>
        <span class="log-message">${mensaje}</span>
    `;
    
    // Insertar al principio (el contenedor está en reverse)
    logContainer.insertBefore(logEntry, logContainer.firstChild);
    
    // Limitar a 50 entradas
    while (logContainer.children.length > 50) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

// ==================== MODAL ====================
function mostrarModalMQTT() {
    const modal = document.getElementById('mqtt-config-modal');
    if (modal) {
        modal.style.display = 'flex';
    }
}

function ocultarModalMQTT() {
    const modal = document.getElementById('mqtt-config-modal');
    if (modal) {
        modal.style.display = 'none';
    }
}

// ==================== MANEJO DE ERRORES GLOBALES ====================
window.addEventListener('error', function(e) {
    console.error('Error global:', e.error);
    agregarLog('Error en la aplicación: ' + e.message, 'error');
});

// ==================== LIMPIEZA AL CERRAR ====================
window.addEventListener('beforeunload', function() {
    if (mqttClient && mqttClient.connected) {
        mqttClient.end();
    }
});

// ==================== INFORMACIÓN DE DEBUG ====================
console.log('=== Dashboard IoT Invernadero Hidropónico ===');
console.log('Versión: 1.0');
console.log('Broker MQTT:', MQTT_CONFIG.broker);
console.log('Client ID:', MQTT_CONFIG.clientId);
console.log('Topics:', TOPICS);
console.log('=============================================');
