# Clase 74: Integración de **MoveIt 2** con Arduinobot (configuración y archivos)

> **Objetivo de la clase**  
Integrar el robot **Arduinobot** con **MoveIt 2** para planificar trayectorias y mover el efector final (gripper) en el espacio 3D, creando un paquete `arduinobot_moveit` y preparando los archivos de configuración mínimos para cinemática, límites articulares y cartesianas, y el enlace con `ros2_control`.

---

## 1) Pasos ejecutados en terminal (y por qué)

```bash
# 1. Crear el paquete de configuración para MoveIt 2
ros2 pkg create --build-type ament_cmake arduinobot_moveit

# 2. Compilar el workspace para registrar el nuevo paquete
colcon build

# 3. Cargar el entorno del workspace compilado
. install/setup.bash

# 4. (Recomendado por MoveIt 2) Instalar Cyclone DDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# 5. Exportar el RMW (middleware DDS) a Cyclone DDS en tu .bashrc
vim ~/.bashrc
# dentro de .bashrc añade:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 6. (Opcional) Recargar la configuración de la shell actual
source ~/.bashrc

# 7. Alternativa guiada: lanzar el Setup Assistant de MoveIt 2
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Explicación breve:**
- **`ros2 pkg create …`**: genera el esqueleto del paquete `arduinobot_moveit` donde guardamos todos los YAML/SRDF.  
- **`colcon build`** + **`. install/setup.bash`**: compila y expone el paquete al entorno ROS 2.  
- **Cyclone DDS**: MoveIt 2 recomienda esta implementación DDS por compatibilidad/estabilidad; la variable `RMW_IMPLEMENTATION` fuerza su uso por defecto.  
- **Setup Assistant**: herramienta gráfica para generar configs de MoveIt 2 automáticamente (en este curso preferimos una configuración *mínima y legible* hecha a mano).

---

## 2) Estructura sugerida del paquete

```
arduinobot_moveit/
└─ config/
   ├─ arduinobot.srdf
   ├─ initial_positions.yaml
   ├─ joint_limits.yaml
   ├─ kinematic.yaml        # (en tus archivos aparece como "kinematic..yaml")
   ├─ moveit_controllers.yaml
   └─ pilz_cartesian_limits.yaml  # (en tus archivos aparece como "pliz_cartersian_limits.yaml")
```

> Nota: En lo que recibí hay pequeñas diferencias en nombres de archivo:
> - `kinematic..yaml` → **recom:** `kinematic.yaml`  
> - `pliz_cartersian_limits.yaml` → **recom:** `pilz_cartesian_limits.yaml` (Pilz es el planificador industrial de MoveIt)  
> Renombrar evita confusiones y errores de carga.

---

## 3) ¿Qué hace cada archivo y cómo se configuró?

### 3.1 `arduinobot.srdf` — *Semantic Robot Description Format* (semántica para MoveIt)
Define la **estructura semántica** que MoveIt usa para planificar:
- **Grupos** de articulaciones:  
  - `arm`: típicamente incluye `virtual_joint` (si existe) y las **juntas 1–3** del brazo.  
  - `gripper`: incluye **joint_4** y **joint_5** (esta última **mimic** de `joint_4`).
- **Estados iniciales de grupo** (`group_state`), p. ej. `home` con todas las juntas en 0.
- **Pares de links con colisión deshabilitada** (links adyacentes o en contacto permanente) para acelerar la planificación y evitar falsos positivos.
> Esta configuración se redactó manualmente según la transcripción de la clase (grupos `arm` y `gripper`, estado `home`, y la lista de colisiones deshabilitadas entre links adyacentes).

**Buenas prácticas:**
- Asegúrate de que los **nombres de joints/links** en el SRDF **coincidan** exactamente con los del URDF/Xacro.
- Define al menos un `group_state` cómodo: `home` con todas las articulaciones a 0 es perfecto para pruebas.

---

### 3.2 `initial_positions.yaml`
Archivo sencillo con la **configuración articular inicial** (posiciones en radianes) que usarán los controladores al iniciar.
Resumen de lo que contiene:
- Todas las juntas **joint_1…joint_4** en **0 rad** (estado neutro).  
  *(Ver sección de “Posibles correcciones” sobre el nombre de la clave)*

> **Extracto representativo (lo que recibí):**
```yaml
initial position:
  joint_1: 0
  joint_2: 0
  joint_3: 0
  joint_4: 0
```
**Cómo se usa:** El archivo se consulta en los lanzadores/controladores para fijar una postura inicial reproducible antes de planificar o ejecutar trayectorias.

---

### 3.3 `joint_limits.yaml`
Define los **límites cinemáticos por articulación** que MoveIt respeta al planificar:
- **Escalados globales**: velocidad y aceleración por defecto en **0.1** (10%) para movimientos más suaves y seguros.  
- Para cada `joint_1 … joint_5`:
  - `has_velocity_limits: true` con `max_velocity: 10.0`
  - `has_acceleration_limits: false` (se ignora `max_acceleration`)

> **Extracto representativo (lo que recibí):**
```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_sacling_factor: 0.1  # <- ver corrección abajo

joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 10.0
    has_acceleration_limits: false
    max_acceleration: 0.0
  # ... igual para joint_2, joint_3, joint_4, joint_5
```

**Posibles correcciones:**
- **Typo** en `default_acceleration_sacling_factor` → debe ser **`default_acceleration_scaling_factor`**.  
- Si deseas limitar aceleración, pon `has_acceleration_limits: true` y un `max_acceleration` realista.

---

### 3.4 `kinematic.yaml`
Selecciona y ajusta el **solver de cinemática inversa (IK)** para el grupo `arm`:
- **Plugin**: `kdl_kinematics_plugin/KDLKinematicsPlugin` (estable y suficiente para 3 GDL).
- **Resolución de búsqueda IK**: `0.005`
- **Timeout**: `0.005` s
- **Sólo posición**: `position_only_ik: true` (como el brazo tiene 3 GDL, se prioriza la **posición** del efector final y no su orientación).

> **Extracto representativo (lo que recibí):**
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  position_only_ik: True  # recom: usar 'true' en minúsculas
```

**Buenas prácticas:**
- Mantén `position_only_ik: true` mientras el brazo no tenga grados de libertad suficientes para orientar el gripper arbitrariamente.

---

### 3.5 `moveit_controllers.yaml`
Conecta MoveIt 2 con **ros2_control** para ejecutar las trayectorias:
- **Gestor**: `moveit_simple_controller_manager/MoveItSimpleControllerManager`
- **Controladores**:
  - `arm_controller` → `FollowJointTrajectory` sobre `joint_1, joint_2, joint_3`
  - `gripper_controller` → `FollowJointTrajectory` sobre `joint_4, joint_5` (siendo `joint_5` mimic de `joint_4`)

> **Extracto representativo (lo que recibí):**
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  - arm_controller
  - gripper_controller

arm_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: True
  joints:
    - joint_1
    - joint_2
    - joint_3

gripper_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: True
  joints:
    - joint_4
    - joint_5
```

**Posibles mejoras (estructura):**
- Muchas plantillas de MoveIt 2 usan:
  ```yaml
  moveit_simple_controller_manager:
    controller_names:
      - arm_controller
      - gripper_controller
  ```
  Ambas variantes pueden funcionar, pero mantén consistencia con tus launch files.

---

### 3.6 `pilz_cartesian_limits.yaml` *(en tus archivos: `pliz_cartersian_limits.yaml`)*
Límites **cartesianos** (del efector final) usados por el planificador **Pilz** (movimientos *LIN/CIRC*):
- Velocidad traslacional máx.: **1.0**
- Aceleración traslacional máx.: **2.25**
- **Deceleración** traslacional máx.: **-5** (convención, negativa)
- Velocidad rotacional máx.: **1.57** rad/s

> **Extracto representativo (lo que recibí):**
```yaml
cartesian_limits:
  max_trans_vel: 1.0
  max_trans_acc: 2.25
  max_trans_dec: -5
  max_rot_vel: 1.57
```

**Recomendaciones:**
- Renombra el archivo a **`pilz_cartesian_limits.yaml`** y asegúrate de cargarlo en los lanzadores de MoveIt si usarás el planificador Pilz.

---

## 4) Flujo lógico de la configuración

1. **URDF/Xacro** define la geometría/juntas.  
2. **SRDF** agrupa juntas y desactiva colisiones innecesarias → *qué puede planificar MoveIt*.  
3. **`kinematic.yaml`** elige el solver IK (KDL) y su granularidad → *cómo resuelve IK*.  
4. **`joint_limits.yaml`** y **`pilz_cartesian_limits.yaml`** acotan velocidades/aceleraciones → *qué es seguro y válido*.  
5. **`moveit_controllers.yaml`** enlaza con `ros2_control` → *cómo ejecutar lo planificado*.  
6. **Cyclone DDS** mejora la compatibilidad de la comunicación → *que todo hable bien*.

---

## 5) Comprobación rápida

- **Cargar los parámetros** con tus launch files de MoveIt 2 (o con el Setup Assistant).  
- Mover a `home` (todas las juntas en 0) y probar un **goal cartesiano** corto con velocidad reducida.  
- Verifica que el gripper se mueva con `joint_4` y que `joint_5` imite correctamente (mimic).

---

## 6) Posibles correcciones puntuales detectadas

- **Nombres de archivo**:
  - `kinematic..yaml` → `kinematic.yaml`
  - `pliz_cartersian_limits.yaml` → `pilz_cartesian_limits.yaml`
- **YAML booleans**: usar `true/false` en minúsculas para máxima compatibilidad.
- **`initial_positions.yaml`**: la clave aparece como `initial position:` (con espacio). Suele usarse `initial_positions:` (con guion bajo y plural). Ajusta según tus launch files.
- **`joint_limits.yaml`**:
  - `default_acceleration_sacling_factor` → `default_acceleration_scaling_factor`
  - Si quieres limitar aceleraciones, cambia a `has_acceleration_limits: true` y define `max_acceleration`.

---

## 7) Referencias y notas

- Contenido redactado a partir de tu transcripción de clase y de los archivos de configuración suministrados.  
- Se recomienda versionar estos archivos en `arduinobot_moveit/config/` y documentar en el README del repo principal cómo se cargan desde tus launch files.