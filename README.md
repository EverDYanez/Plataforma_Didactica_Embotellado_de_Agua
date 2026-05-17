# Plataforma_Didactica_Embotellado_de_Agua
Integración de una arquitectura MES para una línea de embotellado automatizada usando MQTT, Snap7, Node-RED y PLC Siemens.
# Plataforma Didáctica de Embotellado de Agua - Sistema MES Integrado


## Descripción General

Este proyecto presenta un **prototipo didáctico de un proceso de embotellado de agua potable** integrado con una arquitectura **MES (Manufacturing Execution System)** que implementa estándares internacionales de automatización industrial. Combina una **celda flexible de manufactura** con un **manipulador robótico SCORA-ER 14**, cuatro **estaciones de simulación** (Raspberry Pi) y un **PLC Siemens S7-1200**, demostrando la interoperabilidad entre sistemas físicos y digitales en tiempo real.



## Arquitectura Tecnológica

### Modelo Jerárquico ISA-95

```
┌─────────────────────────────────────────────────────┐
│  Nivel 4: ERP / Planificación Empresarial           │
│          (Funcionalidad futura)                      │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  Nivel 3: MES (Manufacturing Execution System)      │
│  └─ Libre MES (Grafana + InfluxDB + PostgreSQL)     │
│  └─ Node-RED (Middleware de Integración)            │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  Nivel 2: Supervisión y Control                     │
│  └─ PLC Siemens S7-1200 (Lógica GRAFCET/PackML)    │
│  └─ 4x Raspberry Pi 3B+ (Simulaciones Pygame)       │
│  └─ HMI/SCADA (Dashboards Grafana)                  │
└──────────────────┬──────────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────────┐
│  Nivel 1: Actuación y Sensado                       │
│  └─ Robot SCORA-ER 14 (Manipulador industrial)      │
│  └─ Banda transportadora (Motores 24VDC)            │
│  └─ Sensores de presencia (Finales de carrera)      │
└─────────────────────────────────────────────────────┘
```

---

## Componentes Principales

### 1. **Hardware de Control**
| Componente | Modelo | Función |
|-----------|--------|---------|
| **PLC** | Siemens S7-1200 | Control de banda transportadora y lógica GRAFCET |
| **Raspberry Pi** | 3B+ (x4) | Simulaciones digitales de etapas (Pygame) |
| **Robot Industrial** | SCORA-ER 14 | Manipulador SCARA para tareas de ensamble |
| **Sensores** | Finales de carrera | Detección de presencia de botellas |

### 2. **Protocolos de Comunicación**

```
┌──────────────────────┐
│   Raspberry Pi 1     │
│  (Etapa: Lavado)     │──┐
└──────────────────────┘  │
                          │
┌──────────────────────┐  │     ┌─────────────────────┐
│   Raspberry Pi 2     │  │     │   PLC S7-1200       │
│  (Etapa: Llenado)    │──┤────▶│  Control de Banda   │
└──────────────────────┘  │     └─────────────────────┘
                          │
┌──────────────────────┐  │     ┌─────────────────────┐
│   Raspberry Pi 3     │  │     │   MQTT Broker       │
│  (Etapa: Taponado)   │──┤────▶│  (Mosquitto)        │
└──────────────────────┘  │     └─────────────────────┘
                          │
┌──────────────────────┐  │     ┌─────────────────────┐
│   Raspberry Pi 4     │  │     │   Node-RED          │
│  (Etapa: Etiquetado) │──┘     │  (Middleware)       │
└──────────────────────┘        └──────────┬──────────┘
                                           │
                ┌──────────────────────────┼──────────────────────────┐
                │                          │                          │
         ┌──────▼──────┐          ┌────────▼────────┐        ┌───────▼──────┐
         │  InfluxDB   │          │   PostgreSQL    │        │  SCORA-ER 14 │
         │ (Series de  │          │  (Órdenes de    │        │    (TCP/IP)  │
         │   tiempo)   │          │   producción)   │        └──────────────┘
         └──────┬──────┘          └────────┬────────┘
                │                          │
         ┌──────▼──────────────────────────▼──────┐
         │  Libre MES (Grafana + PostgREST)       │
         │  Dashboards OEE y Métricas             │
         └───────────────────────────────────────┘
```

### 3. **Protocolos Utilizados**

- **MQTT**: Publicación/Suscripción entre Raspberry Pi y Node-RED (bajo consumo de recursos)
- **Snap7**: Comunicación PLC ↔ Raspberry Pi sobre TCP/IP (protocolo S7 de Siemens)
- **TCP/IP**: Comunicación con robot SCORA-ER 14 (análisis de ingeniería inversa)
- **REST API**: Acceso a PostgreSQL mediante PostgREST

---

## 📊 Etapas del Proceso de Embotellado

### 1. **Lavado Interior**
- Sanitización de botellas con solución a presión
- Enjuague con agua suavizada
- Validación: sensor de presencia

### 2. **Llenado**
- Llenado de botellas con agua purificada
- Control de volumen
- Representación digital: animación de flujo con partículas

### 3. **Taponado**
- Posicionamiento y sellado de tapas
- Integración con robot SCORA-ER 14
- Detección de éxito mediante sensor

### 4. **Etiquetado**
- Aplicación de etiquetas de información
- Control de posicionamiento
- Verificación de completitud

---

## 🛠️ Tecnologías Implementadas

### Stack Tecnológico

| Capa | Tecnología | Propósito |
|------|-----------|----------|
| **Visualización** | Grafana | Dashboards OEE y KPIs |
| **Base de Datos (Series)** | InfluxDB | Historiador de datos operativos |
| **Base de Datos (Relacional)** | PostgreSQL | Órdenes, lotes, maestros |
| **Middleware** | Node-RED | Orquestación y transformación de datos |
| **Broker MQTT** | Mosquitto | Mensajería ligera IoT |
| **Control Industrial** | PLC Siemens S7-1200 | Lógica GRAFCET/PackML |
| **Simulación Visual** | Pygame (Python) | Representaciones digitales 2D |
| **Comunicación PLC** | Snap7 | Protocolo S7 sobre Ethernet |
| **MES** | Libre MES | Código abierto, Apache 2.0 |
| **Lenguaje** | Python | Desarrollo principal |
| **OS** | Raspberry Pi OS (Debian) | Sistema operativo SBC |

### Estándares Implementados

- **ISA-88**: Gestión de procesos por lotes (Batch Control)
- **ISA-95**: Integración de sistemas de control empresarial
- **PackML**: Lenguaje estandarizado para máquinas discretas

---

### Requisitos Previos

**Hardware:**
- PLC Siemens S7-1200 con firmware actualizado
- 4x Raspberry Pi 3B+ con microSD (≥32GB)
- 4x Pantalla 7" (HDMI, sin táctil)
- Robot SCORA-ER 14 con PLC Yaskawa MP2300S
- Celda flexible de manufactura con banda transportadora
- Computadora para ejecutar Node-RED, InfluxDB, Grafana

**Software:**
- Python 3.8+
- Broker MQTT (Mosquitto)
- Node-RED
- Docker y Docker Compose (para Libre MES)
- TIA Portal (solo para reprogramar PLC, opcional)



### Métricas Capturadas
| Métrica | Unidad | Dashboard |
|---------|--------|-----------|
| **Throughput** | botellas/min | Grafana - Producción |
| **Latencia punto-a-punto** | ms | Grafana - Desempeño |
| **Pérdida de mensajes** | % | Grafana - Confiabilidad |
| **Tiempo de respuesta PLC** | ms | Grafana - Control |
| **Uso CPU Raspberry** | % | Grafana - Recursos |
| **Uso RAM Raspberry** | % | Grafana - Recursos |

### Resultados Obtenidos

- ✅ **Latencia promedio**: 103 ms
- ✅ **Pérdida de mensajes**: 0%
- ✅ **Tiempo respuesta PLC**: 13 ms promedio
- ✅ **Uso CPU**: 28%
- ✅ **Uso RAM**: 10%

---

## Topología MQTT

```
embotelladora/
├── estacion_1/
│   ├── lavado/estado          → idle/execute/held/complete
│   ├── lavado/contadores      → {good: N, reject: M}
│   └── lavado/feedback        → latencia
├── estacion_2/
│   ├── llenado/estado
│   ├── llenado/contadores
│   └── llenado/feedback
├── estacion_3/
│   ├── taponado/estado
│   ├── taponado/contadores
│   └── taponado/feedback
├── estacion_4/
│   ├── etiquetado/estado
│   ├── etiquetado/contadores
│   └── etiquetado/feedback
└── scora/
    ├── start                  → comando de inicio
    ├── feedback               → posición articular
    └── status                 → estado del robot
```

---


## Casos de Uso Educativos

1. **Automatización Industrial**: Entender ciclo de control y retroalimentación
2. **Redes Industriales**: Implementar MQTT, Snap7, TCP/IP
3. **Sistemas Embebidos**: Programación Raspberry Pi en tiempo real
4. **MES y Supervisión**: Dashboards, trazabilidad
5. **Estándares**: ISA-88 (batch), ISA-95 (integración), PackML
6. **Ingeniería Inversa**: Análisis de protocolo SCORA-ER 14

---

## Troubleshooting

### PLC No Conecta
```
Error: "Connection refused on 192.168.1.1:102"

Soluciones:
1. Verificar IP del PLC: comando ARP en TIA Portal
2. Verificar S7 Communication habilitado en firmware
3. Revisar firewall de Windows en PC con PLC
4. Reiniciar PLC: desconectar alimentación 10 segundos
```

### MQTT: Mensajes No Llegan
```
Error: "No message received on topic embotelladora/estacion_1"

Soluciones:
1. Verificar broker MQTT activo: mosquitto -v
2. Comprobar IP broker en config Raspberry
3. Revisar firewall puerto 1883
4. Test con: mosquitto_pub/sub -t test -m "hello"
```

### Raspberry Lenta/Congelada
```
Síntoma: Bajo FPS en simulación, CPU 100%

Soluciones:
1. Reducir resolución pantalla o FPS (30 → 20)
2. Detener servicios innecesarios
3. Mejorar dissipación térmica (heatsink)
4. Revisar archivo swap disponible
```


## Contribuciones

Las contribuciones son bienvenidas. Para colaborar:

1. **Fork** el repositorio
2. **Crea** una rama para tu feature: `git checkout -b feature/mi-aportacion`
3. **Commit** tus cambios: `git commit -m 'Agregar: descripción clara'`
4. **Push** a la rama: `git push origin feature/mi-aportacion`
5. **Abre** un Pull Request con descripción detallada

### Áreas de Contribución
- Optimización de código Python
- Mejoras en dashboards Grafana
- Integración de más máquinas
- Documentación y ejemplos
- Pruebas de estrés y robustez

---

##  Recomendaciones Futuras

1. **Sustitución de sensores**: Tecnología capacitiva o inductiva sin contacto
2. **Reemplazo de bandas**: Nuevas bandas transportadoras de material duradero
3. **Integración ERP ligera**: Para gestión de órdenes de producción
4. **Escalabilidad**: Extensión a otros procesos de manufactura discreta
5. **Robustez**: Pruebas de tolerancia a fallos y recuperación
6. **Gemelo digital**: Modelo 3D sincronizado en tiempo real
7. **IA/ML**: Predicción de fallos y optimización automática

---

## Licencia

Este proyecto está bajo la licencia **Apache 2.0**. Consulta el archivo [LICENSE](LICENSE) para más detalles.

> Esto incluye el software de supervisión, código de control y documentación técnica. El robot SCORA-ER 14 y el PLC Siemens S7-1200 mantienen sus propias restricciones de licencia.

---

## Publicación Académica

**Autores**: Juan José Tovar Parra, Ever David Yañez Carrillo  
**Código de Estudiantes**: 2205887, 2210790  
**Director**: Diego Martínez Castro, PhD en Automática, Robótica e Informática Industrial  
**Institución**: Universidad Autónoma de Occidente (UAO)  
**Programa**: Ingeniería Mecatrónica  
**Año**: 2026

**Documento Completo**: [embotella.pdf](embotella.pdf)

---

## Contacto

**Autor**: Ever David Yañez Carrillo  
**GitHub**: [@EverDYanez](https://github.com/EverDYanez)  

**Autor**: Juan José Tovar Parra  
**GitHub**: [@JuanTovar](https://github.com/JJTovar15)  



---

