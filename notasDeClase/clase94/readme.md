# Clase 94 — Modelo de Interacción por Voz con Alexa y ROS 2

> **Objetivo de la clase:** Completar el modelo de interacción por voz de Alexa para controlar el robot **Arduinobot** a través de comandos de voz, conectando la **Skill de Alexa** con el **Action Server de ROS 2** mediante el nodo `alexa_interface.py`. Además, se actualizan los archivos `remote_interface.launch.py` y `CMakeLists.txt` para integrar este nuevo nodo al sistema completo.

---

## 1. Archivo `alexa_interface.py` — Interfaz entre Alexa y ROS 2

Este script combina el servidor web Flask (para recibir solicitudes desde Alexa) con un cliente de acción ROS 2 (`ActionClient`) que envía objetivos al servidor de tareas (`task_server`). Permite que frases como “Pick up that pen” o “Put the robot to sleep” activen diferentes tareas en el robot.

### **Funcionamiento general**

- Se inicia un **servidor Flask** para recibir solicitudes HTTP desde Alexa (usando Ngrok para exponerlo).
- Se crea un **cliente de acción ROS 2** (`ActionClient`) conectado al servidor `task_server`, que utiliza la interfaz `ArduinobotTask`.
- Cada *intent* de Alexa tiene su propio **manejador (handler)**:
  - `LaunchRequestHandler`: activa el robot y ejecuta la tarea 0 (posición inicial).
  - `PickIntentHandler`: ejecuta la tarea 1 (movimiento de agarre).
  - `SleepIntentHandler`: ejecuta la tarea 2 (posición de descanso).
  - `WakeIntentHandler`: ejecuta la tarea 0 (posición inicial al despertar).
  - `AllExceptionHandler`: gestiona errores y respuestas no reconocidas.

### **Pseudocódigo explicativo**

```text
INICIO
  Importar librerías Flask, ASK SDK, rclpy, threading y ArduinobotTask
  Crear hilo paralelo para inicializar rclpy (hilo de ROS 2)
  Crear nodo ROS 2 llamado “alexa_interface”
  Crear cliente de acción que se conecta al servidor “task_server”

  Definir clase LaunchRequestHandler:
    si el tipo de mensaje es “LaunchRequest” → responder “Hi, how can I help”
    enviar goal.task_number = 0 al servidor ROS 2

  Definir clase PickIntentHandler:
    si se activa “PickIntent” → responder “Ok, I am moving”
    enviar goal.task_number = 1 al servidor ROS 2

  Definir clase SleepIntentHandler:
    si se activa “SleepIntent” → responder “Ok, see you later”
    enviar goal.task_number = 2 al servidor ROS 2

  Definir clase WakeIntentHandler:
    si se activa “WakeIntent” → responder “Hi, I am ready”
    enviar goal.task_number = 0 al servidor ROS 2

  Definir clase AllExceptionHandler:
    manejar cualquier excepción → decir “Sorry, I didn’t get it. Can you please say it again?”

  Crear SkillBuilder y registrar todos los handlers
  Crear SkillAdapter para enlazar Flask con la Skill de Alexa
  Registrar ruta “/” para despachar las solicitudes entrantes

  Ejecutar servidor Flask (app.run en localhost:5000)
FIN
```

> 🔹 **Nota:** la ejecución simultánea del servidor Flask y el nodo ROS 2 se logra mediante *threading*, permitiendo que ambos procesos (Alexa y ROS 2) funcionen en paralelo.

---

## 2. Archivo `remote_interface.launch.py` — Lanzamiento del nodo Alexa Interface

Este archivo **añade el nodo `alexa_interface.py`** al sistema de lanzamiento general de `arduinobot_remote`, junto al `task_server` tanto en versión Python como C++.

### **Funcionamiento general**

- Declara argumentos de lanzamiento (`is_sim` y `use_python`) para definir si se ejecuta en simulación o en hardware real.
- Carga la configuración de MoveIt para el robot.
- Crea y lanza tres nodos:
  - `task_server_node`: versión C++ del servidor de tareas.
  - `task_server.py`: versión Python (condicional según `use_python`).
  - `alexa_interface.py`: interfaz de voz Alexa ↔ ROS 2.

### **Pseudocódigo explicativo**

```text
INICIO
  Declarar argumentos: is_sim, use_python
  Crear configuración MoveIt con URDF y SRDF del Arduinobot
  Definir nodo task_server_node (C++)
  Definir nodo task_server_node_py (Python)
  Definir nodo alexa_interface_node (Python → Alexa)
  Agregar todos los nodos al LaunchDescription
  Retornar LaunchDescription (ROS 2 inicia todos los nodos)
FIN
```

> 🔹 **Propósito:** permitir ejecutar simultáneamente la interfaz de voz, el servidor de tareas y el sistema de planificación de movimientos en una sola instrucción de lanzamiento.

---

## 3. Archivo `CMakeLists.txt` — Instalación del nodo Alexa Interface

Este archivo fue modificado para **instalar el script `alexa_interface.py`** dentro del paquete `arduinobot_remote`.

### **Cambios principales**

- Se añadió la línea:
  ```cmake
  INSTALL(PROGRAMS
    ${PROJECT_NAME}/task_server.py
    ${PROJECT_NAME}/alexa_interface.py
    DESTINATION lib/${PROJECT_NAME}
  )
  ```
  para que el ejecutable `alexa_interface.py` se instale junto con el paquete ROS 2.

- Se mantienen dependencias hacia `rclpy`, `rclcpp`, `rclcpp_action`, `moveit_ros_planning_interface`, y `arduinobot_msgs`.

### **Pseudocódigo explicativo**

```text
INICIO
  Definir versión mínima de CMake y nombre del proyecto
  Encontrar dependencias de ROS 2 y MoveIt
  Instalar paquete Python (arduinobot_remote)
  Compilar biblioteca task_server (C++)
  Registrar nodo C++ como componente
  Instalar scripts Python:
    - task_server.py
    - alexa_interface.py
  Instalar carpeta launch/
  Ejecutar pruebas opcionales si están habilitadas
  Empaquetar proyecto con ament_package()
FIN
```

> 🔹 **Resultado:** ahora `alexa_interface.py` puede ser lanzado automáticamente por `ros2 launch arduinobot_remote remote_interface.launch.py`.

---

## 4. Flujo completo del sistema

1. **El usuario dice un comando** a Alexa (por ejemplo: “Alexa, pick up that pen”).  
2. **Alexa envía una solicitud HTTP** al servidor Flask del PC (Ngrok ↔ Flask).  
3. **`alexa_interface.py` interpreta el intent**, crea un `Goal` y lo envía al `task_server`.  
4. **El `task_server` ejecuta la tarea** correspondiente (movimiento, agarre, descanso, etc.).  
5. **El robot responde físicamente**, completando la orden del usuario.  

---

## 5. Próximos pasos

- Integrar todo el sistema dentro del paquete **bringup del Arduinobot**, para iniciar con un solo `launch`.  
- Añadir más intents (por ejemplo: *Open Gripper*, *Move to Camera Position*).  
- Optimizar la sincronización entre Flask y ROS 2 mediante colas o tópicos asíncronos.  
