# Clase 78: Servidor de Acciones Simple en ROS 2 (Fibonacci)

Este proyecto implementa un **servidor de acciones en ROS 2** que calcula la secuencia de Fibonacci, utilizando Python.  
Forma parte del paquete `arduinobot_py_examples` y usa como interfaz la acción `Fibonacci.action` definida en `arduinobot_msgs`.

---

## 📌 Lógica del archivo `simple_action_server.py`

El archivo `simple_action_server.py` contiene un nodo de ROS 2 que actúa como servidor de acciones.  
Este servidor recibe un número entero (`order`) y devuelve la secuencia de Fibonacci hasta dicho orden, publicando retroalimentación periódica.

### 🔹 Pseudocódigo de la lógica

```
INICIAR librerías rclpy, time, ActionServer y Fibonacci.action

CLASE SimpleActionServer extiende Node:
    - Constructor:
        -> Inicializa nodo con nombre "simple_action_server"
        -> Crea un ActionServer con:
            - Tipo de acción: Fibonacci
            - Nombre de la acción: "fibonacci"
            - Función callback: goalCallback
        -> Muestra mensaje "Starting the server"

    - goalCallback(goal_handle):
        -> Imprime el orden solicitado (goal_handle.request.order)
        -> Inicializa mensaje de feedback con [0, 1]
        -> Para cada número hasta el orden solicitado:
            -> Calcula siguiente número = suma de los dos anteriores
            -> Agrega número a la secuencia parcial
            -> Publica feedback al cliente
            -> Pausa 1 segundo (simulación de trabajo)
        -> Marca el goal como completado con éxito
        -> Crea mensaje de resultado con la secuencia completa
        -> Retorna el resultado al cliente

FUNCIÓN main():
    -> Inicializa ROS 2
    -> Crea instancia del servidor de acciones
    -> Mantiene el nodo activo con spin()
    -> Libera recursos y apaga ROS 2
```

---

## 📌 Archivo `Fibonacci.action`

Este archivo define la interfaz de la acción usada por el servidor.  
Contiene tres partes:

1. **Goal (entrada del cliente):**
   ```
   int32 order
   ```
   - Número entero que indica hasta qué orden calcular la secuencia.

2. **Result (salida final):**
   ```
   int32[] sequence
   ```
   - Secuencia completa de Fibonacci calculada.

3. **Feedback (retroalimentación durante el cálculo):**
   ```
   int32[] partial_sequence
   ```
   - Secuencia parcial publicada en cada iteración.

---

## 📌 Archivo `setup.py`【22†source】

En este archivo se declara el paquete Python `arduinobot_py_examples`.  
Los cambios importantes incluyen:

- Inclusión de la entrada en `console_scripts` para que el nodo se pueda ejecutar desde la terminal:
  ```python
  'simple_action_server = arduinobot_py_examples.simple_action_server:main',
  ```
- Permite lanzar el nodo con:
  ```bash
  ros2 run arduinobot_py_examples simple_action_server
  ```

---

## 📌 Archivo `CMakeLists.txt`【23†source】

Pertenece al paquete `arduinobot_msgs` y define la compilación de interfaces.  
Los cambios relevantes fueron:

- Agregar la acción `Fibonacci.action` en la macro `rosidl_generate_interfaces()`:
  ```cmake
  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/AddTwoInts.srv"
    "srv/EulerToQuaternion.srv"
    "srv/QuaternionToEuler.srv"
    "action/Fibonacci.action"
  )
  ```
- Esto asegura que la acción sea generada y pueda usarse en otros paquetes.

---

## 📌 Archivo `package.xml`【24†source】

En este archivo se especifican las dependencias necesarias para trabajar con acciones:

- Se añadió la dependencia a `action_msgs`:
  ```xml
  <depend>action_msgs</depend>
  ```

- También se mantiene la dependencia a `rosidl_default_generators` para la generación de interfaces.

Esto permite que ROS 2 reconozca y compile el archivo `Fibonacci.action`.

---

## 📌 Comandos usados en la práctica

### 🔹 Terminal 1 (lanzar servidor)
```bash
colcon build
. install/setup.bash
ros2 run arduinobot_py_examples simple_action_server
```
- Compila el workspace.
- Fuentea el entorno para reconocer los nuevos paquetes.
- Ejecuta el nodo servidor de acciones.

### 🔹 Terminal 2 (interactuar con el servidor)
```bash
. install/setup.bash
ros2 node list
ros2 action list
ros2 action info /fibonacci -t
ros2 action send_goal /fibonacci arduinobot_msgs/action/Fibonacci "{order: 10}" -f
```
- Verifica los nodos activos (`/simple_action_server`).
- Lista las acciones disponibles (`/fibonacci`).
- Inspecciona la acción y sus servidores.
- Envía un **goal** con orden 10 y recibe retroalimentación progresiva + resultado final.

---

## ✅ Resumen

Este proyecto muestra cómo:

1. **Definir una acción en ROS 2** (`Fibonacci.action`).  
2. **Configurar los archivos de compilación** (`CMakeLists.txt`, `package.xml`, `setup.py`).  
3. **Implementar un servidor de acciones en Python** (`simple_action_server.py`).  
4. **Compilar y probar el sistema** usando dos terminales (servidor y cliente).

---

✍️ Autor: **Johan Alejandro López Arias**