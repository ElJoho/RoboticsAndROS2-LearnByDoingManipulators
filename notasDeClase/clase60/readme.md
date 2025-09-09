# Clase 60 — **TF2 Tools** (ROS 2)

Este `README.md` resume y **explica los comandos usados en clase** para explorar las herramientas de TF2: visualización de marcos de referencia, inspección de tópicos `/tf` y `/tf_static`, generación del grafo de frames y consulta de transformaciones entre pares de frames.

---

## Objetivos de la sesión
- Lanzar la visualización del robot y sus frames en RViz.
- Inspeccionar qué nodos publican/suscriben a `/tf` y `/tf_static`.
- Leer (echo) los mensajes TF para entender su **estructura** (traducción y rotación en **cuaterniones**).
- Generar un **PDF** con el árbol de frames usando `tf2_tools view_frames`.
- Calcular y mostrar la **matriz de transformación 4×4** entre dos frames con `tf2_ros tf2_echo`.
- Ver cómo cambian las transformaciones al mover las articulaciones (Joint State Publisher).

> **Requisito**: Ejecuta cada bloque de comandos en **terminales separadas** y no olvides **sourcear** el `setup.bash` en cada terminal.

---

## 1) Preparación del entorno y visualización en RViz

**Terminal 1** (en la raíz de tu workspace `arduinobot_ws`):

```bash
colcon build
. install/setup.bash
ros2 launch arduinobot_description display.launch.py
```

**¿Qué hace cada comando?**

- `colcon build`: compila todos los paquetes del workspace.
- `. install/setup.bash`: añade al entorno los paths de instalación para que ROS 2 encuentre los paquetes recién compilados.
- `ros2 launch arduinobot_description display.launch.py`: lanza la visualización del robot (RViz) con TF/Joint State Publisher configurados para publicar los frames.

> **Nota**: En los apuntes aparece `ros2 launch arduino` (incompleto). El comando correcto para esta clase es `ros2 launch arduinobot_description display.launch.py`.

---

## 2) Exploración de tópicos TF

**Terminal 2**:

```bash
. install/setup.bash
ros2 topic list
```

**Propósito**: listar los tópicos activos. Deberías ver, entre otros:  

```
/tf
/tf_static
/joint_states
/robot_description
```

---

## 3) ¿Quién publica en `/tf`? (info y verbose)

Sigue en **Terminal 2**:

```bash
ros2 topic info /tf
ros2 topic info /tf --verbose
```

**Qué observamos**:
- `robot_state_publisher` suele ser el **publisher** de `/tf` (dinámico).
- Un `transform_listener` suele figurar como **subscriber**.
- Con `--verbose` se ven **QoS**, **GID**, tipo de mensaje (`tf2_msgs/msg/TFMessage`), etc.

---

## 4) Leer el contenido de `/tf` (echo)

```bash
ros2 topic echo /tf
```

**Interpretación**:
- Cada mensaje incluye una **lista de transformaciones** (`transforms`).
- Cada transformación tiene `header`, `child_frame_id`, `transform.translation (x,y,z)` y `transform.rotation (x,y,z,w)`.
- **Rotación** en TF2/ROS 2 se expresa por convención en **cuaterniones** `(x, y, z, w)`.
- La secuencia de transforms define la posición/orientación de cada frame respecto a su **padre**.

Puedes detener el `echo` con `Ctrl+C` para continuar.

---

## 5) Grafo de frames en PDF (`tf2_tools view_frames`)

```bash
ros2 run tf2_tools view_frames
ls
# (opcional) Abrir el PDF en Linux:
evince frames_YYYY-MM-DD_HH.MM.SS.pdf
```

**Qué hace**:
- Escucha ~5 s los datos de TF y genera un **grafo de frames**.
- Produce dos archivos: `frames_*.gv` y `frames_*.pdf` en el directorio actual.

**Cómo leer el PDF**:
- Los nodos son **frames** (e.g., `world`, `base_link`, `base_plate`, `horizontal_arm`, `claw_support`, `gripper_left/right`).
- Las aristas indican relaciones **padre–hijo**.
- La **tasa media** (rate) suele ser ~10 Hz para dinámicos; los **estáticos** aparecen con `10000.000` (indicando “no varía”).

> Si no aparece el PDF, verifica que `/tf` y/o `/tf_static` estén activos y vuelve a ejecutar el comando cuando RViz ya esté publicando frames.

---

## 6) Transformación entre dos frames (`tf2_ros tf2_echo`)

Ejemplos usados en clase (en **Terminal 2**):

```bash
ros2 run tf2_ros tf2_echo base_link horizontal_arm
```

Salida típica (resumen):
- **Translation**: `[tx, ty, tz]`
- **Rotation** (Quaternion): `(x, y, z, w)`
- **Rotation** (RPY): en **radianes** y **grados**
- **Matrix 4×4**: matriz homogénea de la transformación (R|t)

Segundo ejemplo (composición a través de la cadena de frames):

```bash
ros2 run tf2_ros tf2_echo world claw_support
```

**Idea clave**: TF2 compone internamente las transformaciones intermedias siempre que exista una **cadena** que conecte los frames (resolviendo de facto la **cinemática directa** para la pose solicitada).

---

## 7) Ver la actualización en tiempo real (mover articulaciones)

Con `display.launch.py` activo, el **Joint State Publisher** ya está corriendo. Al mover los deslizadores de las articulaciones en RViz/GUI:
- Cambian los valores en `/tf`.
- `tf2_echo` actualiza **traducción**, **RPY** y **matriz** en tiempo real.
- La pose del gripper/efector final se recalcula continuamente.

---

## 8) Resumen de comandos utilizados

```bash
# Compilación y entorno
colcon build
. install/setup.bash

# Visualización del robot y frames en RViz
ros2 launch arduinobot_description display.launch.py

# Exploración de tópicos TF
ros2 topic list
ros2 topic info /tf
ros2 topic info /tf --verbose
ros2 topic echo /tf

# Grafo de frames (genera frames_*.pdf y frames_*.gv)
ros2 run tf2_tools view_frames
evince frames_YYYY-MM-DD_HH.MM.SS.pdf   # (opcional, Linux)

# Transformaciones entre dos frames (matriz 4x4 incluida)
ros2 run tf2_ros tf2_echo base_link horizontal_arm
ros2 run tf2_ros tf2_echo world claw_support

# Utilidad general
ls
Ctrl+C   # detener procesos en foreground
```

---

## 9) Buenas prácticas y solución de problemas

- **Sourcéal** `install/setup.bash` en **cada terminal** antes de usar `ros2 ...`.
- Si `view_frames` no genera el PDF, asegúrate de que RViz/`robot_state_publisher` esté publicando `/tf` y repite el comando tras ~5–10 s.
- Si `tf2_echo` no muestra datos, revisa que los nombres de frames sean **exactos** (usa el PDF y RViz para confirmarlos).
- Si ves solo `/tf_static`, puede que falte el **publisher dinámico** (p.ej., `robot_state_publisher`) o que el robot esté parado sin Joint State Publisher.

---

## 10) Artefactos generados durante la clase
- `frames_YYYY-MM-DD_HH.MM.SS.gv`
- `frames_YYYY-MM-DD_HH.MM.SS.pdf`
- Carpetas del workspace: `build/`, `install/`, `log/`