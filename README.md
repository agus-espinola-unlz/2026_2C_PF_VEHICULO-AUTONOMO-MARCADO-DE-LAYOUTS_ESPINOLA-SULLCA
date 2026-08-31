![Logo Institucional](https://github.com/JonatanBogadoUNLZ/PPS-Jonatan-Bogado/blob/9952aac097aca83a1aadfc26679fc7ec57369d82/LOGO%20AZUL%20HORIZONTAL%20-%20fondo%20transparente.png)

# UNLZ — Facultad de Ingeniería (Plantilla de Proyecto)
## Ingeniería Mecatrónica — README + estructura estándar

Este repositorio es una **PLANTILLA**.  
Los estudiantes deben **usar este repo como base** (fork o “Use this template”) y **reemplazar los textos entre corchetes** `[ ... ]` con la información real de su proyecto.

---

## 📛 Naming del repositorio (OBLIGATORIO)

El nombre del repositorio debe seguir este esquema:

**`ANIO_CUATRIMESTRE_TIPO_PROYECTO_APELLIDOS`**

Donde:
- **ANIO**: año de cursada (ej. `2026`)
- **CUATRIMESTRE**: `1C` o `2C`
- **TIPO**: `PPS` o `PF` (Proyecto Final)
- **PROYECTO**: nombre corto *sin espacios* (recomendado: `kebab-case` o `CamelCase`)
- **APELLIDOS**: apellidos de integrantes separados por `_` (sin tildes, sin ñ)

✅ Ejemplos:
- `2026_1C_PPS_ComederoSmart_Salto_Vazquez`
- `2026_2C_PF_MecaChess_Duarte_Diaz`
- `2025_2C_PPS_Escaner3D_DalleRivePrieto_Labreniuk`

> Nota: GitHub **no permite** usar “/” en el nombre del repositorio.  
> Por eso se usa **TIPO = PPS o PF** como campo separado.

---

## 🧩 Cómo usar esta plantilla (estudiantes)

0) **Crear el repo con el nombre correcto (OBLIGATORIO)**  
   Esquema: `ANIO_CUATRIMESTRE_TIPO_PROYECTO_APELLIDOS`

1) Crear tu repositorio desde esta plantilla:
   - Opción A (recomendada): **Use this template** → Create a new repository  
   - Opción B: **Fork**

2) Editar este archivo `README.md` completando todos los campos `[ ... ]`.

3) Subir archivos a las carpetas correspondientes:
   - Código en `CODIGO/`
   - Planos y esquemas en `PLANOS/`
   - Fotos / videos en `MULTIMEDIA/`
   - Datasheets en `DATASHEET/`
   - Informes en `INFORMES/`

---

## ✅ Checklist de entrega
- [ ] Naming correcto del repo: `ANIO_CUATRIMESTRE_TIPO_PROYECTO_APELLIDOS`
- [ ] Título, autores, materia, **tipo (PPS/PF)**, año y cuatrimestre completos
- [ ] Brief completo (one-liner + pitch + problema + solución + alcance + estado)
- [ ] Instrucciones de uso reproducibles (otro puede correrlo)
- [ ] Lista de componentes con cantidades y modelos
- [ ] Esquemáticos/planos adjuntos en `PLANOS/`
- [ ] Fotos / video demostración en `MULTIMEDIA/`
- [ ] Informe PDF en `INFORMES/` (si aplica)

---

# [TRACEBOT]

**Tipo:** [PF]  
**Año:** [2026] — **Cuatrimestre:** [1C]  

**Carrera:** Ingeniería Mecatrónica  
**Materia / Curso:** [Proyecto en Ingenieria Mecatrónica]  
**Docente / Cátedra:** [Ezequiel Blanca, Juan Ignacio Szombach, Cristian Lukaszewicz]  
**Autor/es:** [ESPINOLA, Agustin — Legajo] · [SULLCA, Marco — Legajo]

---

## Introducción / Objetivo

**Contexto (2–4 líneas):**  
[En industrias como la construcción, la manufactura y la logística, la automatización está transformando los procesos para mejorar la eficiencia y la precisión. El replanteo, que consiste en transferir planos digitales al suelo, es una etapa crítica que tradicionalmente se realiza mediante métodos manuales.]

**Problema a resolver:**  
[A nivel nacional existe una importante brecha tecnológica, ya que el replanteo convencional sigue siendo manual, propenso a errores humanos, costoso y dependiente de soluciones importadas. Esta desconexión entre el diseño digital (CAD/BIM) y su ejecución en el terreno genera pérdida de precisión geométrica, retrasos en los tiempos de ejecución y un desperdicio significativo de recursos.]

**Objetivo general:**  
[Diseñar y desarrollar un prototipo de vehículo autónomo alimentado por baterías capaz de automatizar el proceso de marcado de trazados y layouts sobre superficies horizontales planas, a partir de la carga inalámbrica de archivos digitales (CAD/CSV), reduciendo los tiempos de ejecución y eliminando errores de marcado.]

**Objetivos específicos (opcional):**
- [Objetivo 1]
- [Objetivo 2]
- [Objetivo 3]

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
- [Autor](#autor)
- [Licencia](#licencia)

---

## Brief

**One-liner (1 frase):**  
[Un vehículo autónomo de replanteo que automatiza la transferencia de planos digitales al suelo para los sectores de la construcción y logística, eliminando errores de marcado manual y optimizando los tiempos de obra.]

**Elevator pitch (30 segundos):**  
Este proyecto, [TraceBot] (PF, 2026 1er cuatrimestre) , resuelve la desconexión, ineficiencia y pérdida de precisión que ocurren al transferir planos digitales de forma manual al terreno. Mediante un vehículo autónomo móvil de tracción diferencial capaz de realizar trazados rectos y definidos controlados digitalmente , el sistema está orientado a industrias de la construcción, plantas industriales y centros logísticos. Permite reducir significativamente los tiempos de ejecución y garantizar una precisión en el trazado de ±3%. Se implementa con tecnologías clave como el microcontrolador ESP32-WROOM, encoders magnéticos AS5600, giroscopio AS5600 y posicionamiento mediante AprilTags. Se valida mediante una demostración funcional en superficies planas y niveladas de hasta 2×2 metros , verificando el resultado mediante el procesamiento de imágenes por cámara a bordo

### Problema
- **Contexto:** [Obras en construcción, plantas industriales, manufactura y centros logísticos donde se realiza la transferencia de planos digitales al suelo (replanteo o marcado de layout).]
- **Dolor principal:** [El replanteo continúa realizándose a nivel local con métodos tradicionales y puramente manuales , los cuales son lentos, propensos a errores humanos y altamente dependientes de costosas soluciones importadas.]
- **Impacto:** [Provoca una desconexión crítica entre el diseño en CAD/BIM y la ejecución real , lo que genera retrasos imprevistos, la necesidad de realizar costosos retrabajos, desperdicio de recursos económicos y pérdida de precisión geométrica en la obra final.]

### Solución propuesta
- **Qué hace (features):**
  - [Navegación Autónoma: Se desplaza por sí mismo siguiendo trayectorias         complejas de manera controlada sobre un espacio de hasta 2×2 metros a        partir de un archivo digital de CAD o CSV cargado de forma inalámbrica.]
  - [Marcado Preciso: Activa y desactiva de forma inteligente un sistema          físico de marcado con tinta para trazar líneas rectas continuas solo en      las zonas indicadas por el plano.]
  - [Percepción y Seguridad: Detecta obstáculos en tiempo real en su              recorrido deteniendo el movimiento inmediatamente , evalúa la calidad        del trazo mediante una cámara a bordo e indica de forma sonora y             luminosa sus estados de operación (alertas de batería, tinta o fallas).]
- **Cómo lo hace (alto nivel):** [El usuario carga el plano CAD que desea trazar a través de la interfaz; el software procesa el archivo, extrae la trayectoria que debe realizar el vehiculo y comienza a enviar las intrucciones secuencialmente al microcontrolador. Utilizando la cámara externa y los tags, el vehiculo logra reconocer su posición inicial. A medida que las intrucciones llegan al microcontrolador ESP32, este las ejecuta coordinando los motores de tracción para el desplazamiento y accionando la bomba de tinta cuando corresponde. En simultaneo una cámara a bordo visualiza e inspecciona el trazado permitiendo validar que se este realizando de forma correcta y con la calidad requerida]
- **Valor diferencial:** [Representa una solución de desarrollo nacional y de bajo costo relativo ($200 USD en componentes críticos) frente a las herramientas comerciales importadas ($1500 USD por dia, Dusty Robotics) , unificando la precisión del diseño CAD con el terreno físico de forma accesible para el contexto local]

### Alcance
**Incluye:**
- [X]
- [Y]

**No incluye (por ahora):**
- [A]
- [B]

### Estado del proyecto
- **Madurez:** [idea / prototipo / MVP / validado]
- **Qué funciona hoy:** [lista corta]
- **Próximos pasos:** [lista corta]

### Demo rápida
- **Video / GIF:** [link o ruta en MULTIMEDIA]
- **Instrucciones express (2 minutos):**
  1) [Paso 1]
  2) [Paso 2]
  3) [Paso 3]

---

## Descripción técnica
[Explicación técnica del funcionamiento, decisiones de diseño y consideraciones.]

---

## Arquitectura del sistema

**Entradas (sensores / señales):**
- [Sensor 1]
- [Sensor 2]

**Procesamiento / Control:**
- [Microcontrolador / PC / algoritmo / lógica]

**Salidas (actuadores / señales):**
- [Actuador 1]
- [Actuador 2]

**Interfaz (si aplica):**
- [Pantalla / dashboard / app / web]

> (Opcional) Insertar diagrama:
![Diagrama de bloques](PLANOS/diagrama_bloques.png)

---

## Instrucciones de uso

### Requisitos previos
- [Software / IDE]
- [Drivers / librerías]
- [Hardware mínimo]

### Instalación / Puesta en marcha
1) [Clonar / descargar]
2) [Instalar dependencias]
3) [Cargar firmware / ejecutar]
4) [Validar funcionamiento]

### Uso
- **Modo normal:** [cómo se usa]
- **Calibración (si aplica):** [pasos]
- **Notas:** [cuidados, recomendaciones]

### Troubleshooting (opcional)
- **Problema:** [X] → **Solución:** [Y]
- **Problema:** [X] → **Solución:** [Y]

---

## Tecnologías utilizadas
- **Robótica / Control:** [Arduino / ESP32 / Raspberry / etc.]
- **Electrónica:** [sensores / drivers / etc.]
- **Programación:** [C/C++ / Python / etc.]
- **Plataformas / Tools:** [ROS / OpenCV / etc.]
- **IA (si aplica):** [modelo / técnica]

---

## Listado de componentes

| Componente | Cantidad | Modelo / Especificación | Función |
|---|---:|---|---|
| [Componente 1] | [1] | [Modelo] | [Función] |
| [Componente 2] | [2] | [Modelo] | [Función] |
| [Componente 3] | [1] | [Modelo] | [Función] |

---

## Esquemáticos / Planos
- [Plano/Esquemático 1] → `PLANOS/[archivo]`
- [Plano/Esquemático 2] → `PLANOS/[archivo]`

---

## Fotos / Videos
- Foto 1 → `MULTIMEDIA/[archivo]`
- Foto 2 → `MULTIMEDIA/[archivo]`
- Video demo → `MULTIMEDIA/[archivo]` o [link]

---

## Estructura del repositorio
- `CODIGO/` — Código fuente del proyecto.
- `MULTIMEDIA/` — Imágenes y videos.
- `PLANOS/` — Esquemáticos y diagramas.
- `DATASHEET/` — Hojas de datos y especificaciones.
- `INFORMES/` — Informes, Gantt, manuales, PDFs.

---

## Autor
**[APELLIDO, Nombre]** — [Legajo]  
Contacto (opcional): [mail / LinkedIn]

---

## Licencia
[Definir según la cátedra: MIT / uso académico / etc.]

---

## About (descripción corta del repositorio)

Usar este texto (o similar) en el campo **About** de GitHub:

**[PPS | PF] — [Proyecto] — FI-UNLZ — [2026] [1C|2C] — [Apellido1, Apellido2]**
