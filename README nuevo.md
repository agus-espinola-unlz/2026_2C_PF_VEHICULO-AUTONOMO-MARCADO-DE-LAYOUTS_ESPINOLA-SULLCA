![Logo Institucional](https://github.com/JonatanBogadoUNLZ/PPS-Jonatan-Bogado/blob/9952aac097aca83a1aadfc26679fc7ec57369d82/LOGO%20AZUL%20HORIZONTAL%20-%20fondo%20transparente.png)

# TraceBot

**Tipo:** PF — Proyecto Final  
**Año:** 2026 — **Cuatrimestre:** 1C

**Carrera:** Ingeniería Mecatrónica  
**Materia / Curso:** Proyecto en Ingeniería Mecatrónica  
**Docente / Cátedra:** Ezequiel Blanca · Juan Ignacio Szombach · Cristian Lukaszewicz  
**Autor/es:** ESPINOLA, Agustín — [Legajo] · SULLCA, Marco — [Legajo]

---

## ✅ Checklist de entrega
- [x] Naming correcto del repo: `2026_1C_PF_TraceBot_Espinola_Sullca`
- [x] Título, autores, materia, **tipo (PF)**, año y cuatrimestre completos
- [x] Brief completo (one-liner + pitch + problema + solución + alcance + estado)
- [x] Instrucciones de uso reproducibles (otro puede correrlo)
- [x] Lista de componentes con cantidades y modelos
- [ ] Esquemáticos/planos adjuntos en `PLANOS/`
- [ ] Fotos / video demostración en `MULTIMEDIA/`
- [ ] Informe PDF en `INFORMES/` (si aplica)

---

## Introducción / Objetivo

**Contexto:**  
En industrias como la construcción, la manufactura y la logística, la automatización está transformando los procesos para mejorar la eficiencia y la precisión. El replanteo —transferir planos digitales al suelo— es una etapa crítica que tradicionalmente se realiza mediante métodos manuales.

**Problema a resolver:**  
A nivel nacional existe una importante brecha tecnológica: el replanteo convencional sigue siendo manual, propenso a errores humanos, costoso y dependiente de soluciones importadas. Esta desconexión entre el diseño digital (CAD/BIM) y su ejecución en el terreno genera pérdida de precisión geométrica, retrasos en los tiempos de ejecución y desperdicio significativo de recursos.

**Objetivo general:**  
Diseñar y desarrollar un prototipo de vehículo autónomo de tracción diferencial, alimentado por baterías, capaz de automatizar el proceso de marcado de trazados y layouts sobre superficies horizontales planas, a partir de la carga inalámbrica de archivos digitales (.DXF), reduciendo los tiempos de ejecución y eliminando errores de marcado.

**Objetivos específicos:**
- Implementar un sistema de odometría basado en encoders magnéticos AS5600 y giroscopio MPU6050 para la navegación autónoma del vehículo con una precisión de trazado de ±3%.
- Desarrollar una interfaz de escritorio en Python que permita la carga de archivos .DXF, el control manual del vehículo, la gestión del sistema de marcado y la comunicación WiFi con la ESP32.
- Implementar un sistema de localización absoluta mediante AprilTags y cámara externa (smartphone) para determinar la posición inicial del AGV y corregir la deriva acumulada de odometría.
- Integrar una ESP32-CAM a bordo del vehículo para el control de calidad automático del trazado, validando que las líneas marcadas cumplan con los parámetros configurados.

---

## Índice
- [Brief](#brief)
- [Descripción técnica](#descripción-técnica)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Instrucciones de uso](#instrucciones-de-uso)
- [Tecnologías utilizadas](#tecnologías-utilizadas)
- [Listado de componentes](#listado-de-componentes)
- [Esquemáticos / Planos](#esquemáticos--planos)
- [Fotos / Videos](#fotos--videos)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Autores](#autores)
- [Licencia](#licencia)

---

## Brief

**One-liner:**  
Un vehículo autónomo de replanteo que automatiza la transferencia de planos digitales al suelo para los sectores de construcción y logística, eliminando errores de marcado manual y optimizando los tiempos de obra.

**Elevator pitch (30 segundos):**  
TraceBot (PF, 2026 1C) resuelve la desconexión, ineficiencia y pérdida de precisión que ocurren al transferir planos digitales al terreno de forma manual. Mediante un vehículo autónomo de tracción diferencial controlado digitalmente, el sistema está orientado a industrias de la construcción, plantas industriales y centros logísticos. Permite reducir significativamente los tiempos de ejecución y garantizar una precisión en el trazado de ±3%. Se implementa con un microcontrolador ESP32-WROOM, encoders magnéticos AS5600, giroscopio MPU6050 y localización mediante AprilTags. Se valida mediante una demostración funcional en superficies planas de hasta 2×2 metros, verificando el resultado con una cámara a bordo (ESP32-CAM).

### Problema
- **Contexto:** Obras en construcción, plantas industriales, manufactura y centros logísticos donde se realiza la transferencia de planos digitales al suelo (replanteo o marcado de layout).
- **Dolor principal:** El replanteo continúa realizándose a nivel local con métodos tradicionales y puramente manuales, los cuales son lentos, propensos a errores humanos y altamente dependientes de costosas soluciones importadas.
- **Impacto:** Provoca una desconexión crítica entre el diseño en CAD/BIM y la ejecución real, lo que genera retrasos imprevistos, la necesidad de realizar costosos retrabajos, desperdicio de recursos económicos y pérdida de precisión geométrica en la obra final.

### Solución propuesta

**Qué hace (features):**
- **Navegación Autónoma:** Se desplaza por sí mismo siguiendo trayectorias complejas de manera controlada sobre un espacio de hasta 2×2 metros, a partir de un archivo .DXF cargado de forma inalámbrica desde la interfaz.
- **Marcado Preciso:** Activa y desactiva de forma inteligente la bomba peristáltica de tinta para trazar líneas continuas solo en las zonas indicadas por el plano.
- **Localización con AprilTags:** Un tag fijo en el entorno y otro montado sobre el vehículo, detectados por una cámara de smartphone externa, permiten determinar la posición inicial del AGV y corregir la deriva de odometría.
- **Control de Calidad Embarcado:** Una ESP32-CAM a bordo inspecciona el trazado en tiempo real y valida que se esté realizando con la calidad configurada.
- **Interfaz de Control Completa:** Aplicación de escritorio en Python que permite conectar al vehículo por WiFi, cargar el plano, cebar la bomba, realizar movimientos manuales, configurar tolerancias y monitorear el estado del sistema.

**Cómo lo hace (alto nivel):**  
El usuario carga el archivo .DXF del plano a través de la interfaz en PC; el software procesa el archivo, extrae la trayectoria y la envía secuencialmente por WiFi al microcontrolador ESP32. Usando la cámara del smartphone y los AprilTags, el vehículo determina su posición inicial en el espacio. A medida que las instrucciones llegan a la ESP32, esta las ejecuta coordinando los dos motoreductores de 12V DC mediante odometría diferencial (AS5600 + MPU6050), y acciona la bomba peristáltica cuando el plano lo indica. En simultáneo, la ESP32-CAM inspecciona el trazado para validar la calidad del marcado.

**Valor diferencial:**  
Representa una solución de desarrollo nacional y bajo costo (~$200 USD en componentes críticos) frente a herramientas comerciales importadas como Dusty Robotics (~$1500 USD/día de alquiler), unificando la precisión del diseño CAD con el terreno físico de forma accesible para el contexto local.

### Alcance

**Incluye:**
- Interfaz de escritorio en PC con conexión WiFi al AGV
- Sistema de marcado con tinta por bomba peristáltica
- Alimentación autónoma por batería de Li-Ion 18650 (3S)
- Admite archivos `.DXF`
- Cámara externa (smartphone) para localización del AGV mediante AprilTags
- Cámara a bordo (ESP32-CAM) para control de calidad del marcado
- Superficie de trabajo máxima de 2×2 m
- Recalibración de la cámara de calidad si es necesario
- Control manual del vehículo desde la interfaz

**No incluye (por ahora):**
- Localización y precisión de grado industrial
- Capacidad de subir pendientes o circular sobre tierra, barro o superficies resbaladizas
- Capacidad de operar en oscuridad o deslumbramiento
- Escalado a superficies mayores a 2×2 m

### Estado del proyecto
- **Madurez:** Prototipo
- **Qué funciona hoy:** Comunicación WiFi ESP32 ↔ interfaz, control manual de motores, lectura de encoders AS5600 vía TCA9548A, activación de bomba peristáltica, detección de AprilTags con cámara externa, carga y procesamiento de archivos .DXF.
- **Próximos pasos:** Integración completa de la odometría diferencial con corrección por AprilTags, cierre del lazo de control de trayectoria, validación de precisión ±3% en superficie de prueba, implementación del módulo de control de calidad con ESP32-CAM.

### Demo rápida
- **Video / GIF:** `MULTIMEDIA/demo_tracebot.mp4` *(pendiente de carga)*
- **Instrucciones express (2 minutos):**
  1. Encender el vehículo y conectar la PC a la misma red WiFi que la ESP32.
  2. Abrir la interfaz Python, ingresar la IP de la ESP32 y conectar.
  3. Cargar el archivo `.dxf` del plano que se desea trazar.
  4. Cebar la bomba de tinta desde la interfaz.
  5. Iniciar el recorrido.  
  *(Opcional: configurar las tolerancias de calidad y marcado antes de iniciar.)*

---

## Descripción técnica

TraceBot es un AGV (Automated Guided Vehicle) de tracción diferencial compuesto por dos ruedas motrices independientes y una rueda loca. Cada rueda motriz está acoplada a un motorreductor de 12V DC controlado por un driver L298N desde la ESP32-WROOM, que actúa como unidad central de procesamiento y comunicación.

**Odometría y navegación:**  
Cada rueda motriz cuenta con un encoder magnético AS5600 de 12 bits de resolución para medición de velocidad y posición angular. Dado que ambos encoders comparten el bus I2C con la misma dirección de fábrica, se utiliza un multiplexor TCA9548A para habilitarlos de forma independiente en el mismo bus. El giroscopio/acelerómetro MPU6050 provee la medición de ángulo de giro (yaw), complementando la odometría diferencial y corrigiendo errores en los giros del vehículo.

**Localización absoluta:**  
Para evitar la acumulación de error de odometría, se implementa un sistema de localización basado en AprilTags. Un tag de referencia se coloca en una posición fija del entorno y otro tag se monta sobre el vehículo. Una cámara de smartphone apunta al espacio de trabajo y procesa las imágenes para determinar la posición y orientación absoluta del AGV. Esta información se usa para la determinación de la posición inicial y correcciones periódicas de pose.

**Sistema de marcado:**  
Una bomba peristáltica de 12V DC impulsa la tinta a través de un pico en contacto con la superficie. La ESP32 activa y desactiva la bomba según las instrucciones del plano, garantizando el marcado solo en los tramos requeridos.

**Control de calidad:**  
Una ESP32-CAM montada en el chasis del vehículo, orientada hacia la superficie, captura imágenes del trazado en tiempo real. Las imágenes son procesadas para verificar la continuidad y el ancho de las líneas marcadas, comparándolas con los parámetros de tolerancia configurados por el usuario desde la interfaz.

**Alimentación:**  
El sistema es alimentado por un pack de baterías Li-Ion formado por 3 celdas 18650 de 2500 mAh conectadas en serie (configuración 3S), lo que entrega una tensión nominal de 11,1 V (máx. 12,6 V, mín. ~9 V). Esta tensión alimenta directamente los motoreductores y la bomba peristáltica. La ESP32, el MPU6050, los AS5600 y el TCA9548A operan a 3,3 V mediante un regulador de tensión.

**Interfaz de usuario:**  
La aplicación de escritorio, desarrollada en Python, se comunica con la ESP32 por WiFi mediante sockets TCP/UDP. Permite: descubrimiento automático de las IPs de las ESP en la red, carga y visualización del plano .DXF, inicio/pausa/detención del recorrido, control manual del movimiento, cebado de la bomba, configuración de tolerancias de calidad, y visualización del estado del sistema en tiempo real.

---

## Arquitectura del sistema

**Entradas (sensores / señales):**
- 2× Encoder magnético AS5600 (velocidad/posición de cada rueda)
- 1× IMU MPU6050 (ángulo de giro, aceleración)
- AprilTags (localización absoluta vía cámara externa)
- ESP32-CAM a bordo (control de calidad del trazado)

**Procesamiento / Control:**
- ESP32-WROOM (firmware de control: motores, sensores, bomba, WiFi)
- PC con interfaz Python (procesamiento del .DXF, planificación de trayectoria, envío de instrucciones)

**Salidas (actuadores):**
- 2× Motorreductor 12V DC (tracción diferencial)
- 1× Bomba peristáltica 12V DC (marcado con tinta)

**Comunicación:**
- I2C: ESP32 ↔ AS5600 (×2 vía TCA9548A), ESP32 ↔ MPU6050
- WiFi (IEEE 802.11): ESP32 ↔ PC (interfaz Python)
- WiFi (IEEE 802.11): ESP32-CAM ↔ PC (stream de video)

**Interfaz:**
- Aplicación de escritorio Python (Windows/Linux)

> Insertar diagrama de bloques:
![Diagrama de bloques](PLANOS/diagrama_bloques.png)

---

## Instrucciones de uso

### Requisitos previos

**Software (PC):**
- Python 3.10 o superior
- Librerías Python: `ezdxf`, `opencv-python`, `pupil-apriltags`, `numpy`, `tkinter` (incluida en Python), `socket`
- Driver USB-UART CP210x o CH340 (para flashear la ESP32 si es necesario)

**Firmware (ESP32):**
- Arduino IDE 2.x o PlatformIO
- Librerías Arduino: `AS5600`, `MPU6050`, `Wire` (para TCA9548A), `WiFi`, `ESPAsyncWebServer` (o similar)

**Hardware:**
- PC con WiFi (Windows 10/11 o Linux)
- Smartphone con la app de cámara IP (ej. DroidCam o similar) en la misma red WiFi
- Vehículo TraceBot ensamblado con batería cargada y tinta en el depósito

### Instalación / Puesta en marcha

1. **Clonar el repositorio:**
   ```bash
   git clone https://github.com/[usuario]/2026_1C_PF_TraceBot_Espinola_Sullca.git
   cd 2026_1C_PF_TraceBot_Espinola_Sullca
   ```

2. **Instalar dependencias Python:**
   ```bash
   pip install ezdxf opencv-python pupil-apriltags numpy
   ```

3. **Cargar el firmware en la ESP32:**
   - Abrir `CODIGO/firmware_esp32/` en Arduino IDE o PlatformIO.
   - Configurar las credenciales WiFi en `config.h` (SSID y contraseña).
   - Compilar y flashear a la ESP32-WROOM.
   - Repetir para la ESP32-CAM con el sketch correspondiente en `CODIGO/firmware_espcam/`.

4. **Ejecutar la interfaz:**
   ```bash
   python CODIGO/interfaz/main.py
   ```

5. **Validar funcionamiento:**
   - Encender el vehículo; verificar que la ESP32 se conecte a la red WiFi (LED indicador).
   - En la interfaz, ingresar la IP de la ESP32 (o usar el scanner de red integrado) y presionar **Conectar**.
   - Realizar un movimiento manual corto para verificar la comunicación.

### Uso

**Modo normal:**
1. Conectar a la ESP32 desde la interfaz.
2. Cargar el archivo `.dxf` con el plano a trazar.
3. Posicionar el vehículo en el origen (indicado por el AprilTag de referencia en el piso).
4. Cebar la bomba de tinta hasta que fluya de forma continua.
5. Presionar **Iniciar Recorrido**; el vehículo ejecutará el trazado automáticamente.

**Calibración:**
- **Cámara de calidad (ESP32-CAM):** Si las condiciones de iluminación cambian, presionar **Recalibrar Cámara** en la interfaz y seguir el asistente.
- **Posición inicial (AprilTags):** Verificar que el AprilTag de referencia sea visible para la cámara del smartphone antes de iniciar.

**Notas:**
- Operar siempre sobre superficies planas, limpias y sin obstáculos dentro del área 2×2 m.
- Asegurarse de que el nivel de batería sea superior al 20% antes de iniciar un trazado.
- No obstruir la visión de la cámara externa (smartphone) durante la operación.
- Verificar que la tinta no esté obturada antes de comenzar (cebar correctamente).

### Troubleshooting

| Problema | Solución |
|---|---|
| La interfaz no detecta la ESP32 | Verificar que ambos dispositivos estén en la misma red WiFi. Reiniciar la ESP32 y volver a escanear. |
| El vehículo se desvía del trazado | Verificar que los encoders AS5600 estén correctamente montados y que el imán esté centrado. Recalibrar la odometría. |
| La bomba no marca | Cebar nuevamente la bomba. Verificar que haya tinta en el depósito y que la manguera no esté doblada. |
| La cámara de calidad no detecta el trazado | Verificar iluminación y recalibrar la ESP32-CAM desde la interfaz. |
| La ESP32-CAM no aparece en la red | Reiniciar la ESP32-CAM. Verificar credenciales WiFi en el firmware. |

---

## Tecnologías utilizadas

| Categoría | Tecnología / Herramienta |
|---|---|
| **Microcontrolador** | ESP32-WROOM-32, ESP32-CAM |
| **Electrónica** | AS5600 (encoder magnético), MPU6050 (IMU), TCA9548A (mux I2C), driver de motores DC |
| **Firmware** | C/C++ (Arduino framework / PlatformIO) |
| **Interfaz de usuario** | Python 3.x, Tkinter (GUI) |
| **Procesamiento de planos** | Python — ezdxf (parseo .DXF) |
| **Visión computacional** | Python — OpenCV, pupil-apriltags (detección AprilTags y control de calidad) |
| **Comunicación** | WiFi IEEE 802.11, I2C, sockets TCP/UDP |
| **Control de versiones** | Git / GitHub |

---

## Listado de componentes

| Componente | Cantidad | Modelo / Especificación | Función |
|---|---:|---|---|
| Microcontrolador principal | 1 | ESP32-WROOM-32 | Control del vehículo, comunicación WiFi, coordinación de sensores y actuadores |
| Cámara on board | 1 | ESP32-CAM (OV2640) | Control de calidad del trazado a bordo del vehículo |
| Encoder magnético | 2 | AS5600 (12 bits, I2C) | Medición de velocidad y posición angular de cada rueda motriz |
| IMU (giroscopio + acelerómetro) | 1 | MPU6050 (I2C) | Medición del ángulo de giro del vehículo |
| Multiplexor I2C | 1 | TCA9548A (8 canales) | Habilita los 2 encoders AS5600 en el mismo bus I2C |
| Motorreductor 12V DC | 2 | 12V DC c/ reductora (100 RPM) | Tracción diferencial del vehículo |
| Driver de motores | 1 | L298N | Control de velocidad y dirección de los motoreductores |
| Bomba peristáltica | 1 | 12V DC | Impulsión de tinta para el marcado sobre la superficie |
| Celda de batería Li-Ion | 3 | 18650 — 2500 mAh — 3,7V nominal | Alimentación del sistema (configuración 3S: ~11,1V / 2500 mAh) |
| Regulador de tensión | 1 | [3,3V — modelo a completar] | Alimentación de la ESP32, sensores I2C |
| Rueda motriz | 2 | [Diámetro a completar] | Transmisión del movimiento al suelo |
| Rueda loca | 1 | Rueda omnidireccional / caster | Punto de apoyo sin tracción |
| AprilTag (impreso) | 2 | Familia Tag36h11 | Uno fijo en el entorno (referencia), uno sobre el vehículo (localización) |
| Cámara externa | 1 | Smartphone con app de cámara IP | Captura de AprilTags para localización del vehículo |
| Chasis | 1 | [Material / diseño propio] | Estructura mecánica del vehículo |

---

## Esquemáticos / Planos

- Diagrama de bloques del sistema → `PLANOS/diagrama_bloques.png`
- Esquemático electrónico (ESP32 + sensores + drivers) → `PLANOS/esquematico_electronico.pdf`
- Plano mecánico del chasis → `PLANOS/plano_chasis.pdf`
- Diagrama de conexiones I2C (TCA9548A + AS5600 + MPU6050) → `PLANOS/diagrama_i2c.png`

---

## Fotos / Videos

- Vista general del vehículo → `MULTIMEDIA/foto_tracebot_general.jpg`
- Detalle de la electrónica → `MULTIMEDIA/foto_electronica.jpg`
- Detalle del sistema de marcado → `MULTIMEDIA/foto_bomba_tinta.jpg`
- Interfaz de usuario (captura de pantalla) → `MULTIMEDIA/screenshot_interfaz.png`
- Video demostración → `MULTIMEDIA/demo_tracebot.mp4` *(pendiente de carga)*

---

## Estructura del repositorio

```
2026_1C_PF_TraceBot_Espinola_Sullca/
├── CODIGO/
│   ├── firmware_esp32/        # Firmware C/C++ para ESP32-WROOM (control del vehículo)
│   ├── firmware_espcam/       # Firmware C/C++ para ESP32-CAM (control de calidad)
│   └── interfaz/              # Aplicación de escritorio Python
├── MULTIMEDIA/                # Imágenes y videos del proyecto
├── PLANOS/                    # Esquemáticos, diagramas y planos mecánicos
├── DATASHEET/                 # Hojas de datos de componentes (AS5600, MPU6050, TCA9548A, etc.)
└── INFORMES/                  # Informes, Gantt, manuales, PDFs
```

---

## Autores

**ESPINOLA, Agustín** — Legajo: [completar]  
**SULLCA, Marco** — Legajo: [completar]

Contacto: [mail o LinkedIn — opcional]

---

## Licencia

Uso académico — Facultad de Ingeniería, UNLZ.  
Proyecto desarrollado en el marco de la materia *Proyecto en Ingeniería Mecatrónica*, 2026.

---

## About *(campo en GitHub)*

```
PF — TraceBot — FI-UNLZ — 2026 1C — Espinola, Sullca
```
