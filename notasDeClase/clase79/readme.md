# Clase 79 — Servidor de Acciones en C++: `simple_action_server.cpp`

Este documento resume lo realizado en la **Clase 79**, donde implementamos un **Action Server** en C++ para la acción `Fibonacci`, lo integramos al paquete `arduinobot_cpp_examples`, y lo validamos desde la terminal con la CLI de ROS 2. Además, se documentan los cambios en **cada archivo** (`simple_action_server.cpp`, `CMakeLists.txt`, `package.xml`) y se explica el uso de los **comandos** ejecutados en las dos terminales.

---

## 🧩 Objetivo de la clase

- Crear un **servidor de acciones** en C++ que:
  - Reciba una meta con el campo `order` (número de términos).
  - Vaya **publicando retroalimentación** con la secuencia parcial.
  - Entregue la **secuencia completa** como `Result` al finalizar.
- Compilar el proyecto y **ejecutar el nodo**.
- Verificar la acción con la **CLI**: listar, inspeccionar y enviar metas.

---

## 📁 Archivos trabajados

### 1) `simple_action_server.cpp` (lógica del servidor de acciones)

Implementa un nodo `SimpleActionServer` que crea un **Action Server** para la acción `arduinobot_msgs/action/Fibonacci` con los tres *callbacks* clásicos:

- **`goalCallback(uuid, goal)`**: decide aceptar o rechazar la meta (aquí se acepta si `order >= 0`).  
- **`cancelCallback(goal_handle)`**: decide si se permite cancelar (aquí se acepta).  
- **`acceptedCallback(goal_handle)`**: despacha la ejecución asíncrona que calcula la secuencia, publica *feedback* y finalmente responde con *result*.

Durante la compilación se observaron **warnings** (no críticos) por parámetros no usados:
- `uuid` no usado en `goalCallback` (`-Wunused-parameter`).  
- `goal_handle` no usado en `cancelCallback` (`-Wunused-parameter`).  

> Cómo silenciarlos en el futuro (opcional): marcar con `[[maybe_unused]]`, o hacer ` (void)uuid; (void)goal_handle;` dentro de cada callback.

#### 🔎 Pseudocódigo de la lógica (`simple_action_server.cpp`)

```text
INICIAR nodo "simple_action_server"

CREAR action_server para "fibonacci" con callbacks:
  goalCallback(uuid, goal):
    SI goal.order < 0 → RECHAZAR
    EN OTRO CASO → ACEPTAR

  cancelCallback(goal_handle):
    PERMITIR_CANCELACIÓN

  acceptedCallback(goal_handle):
    LANZAR TAREA ASÍNCRONA:
      OBTENER handle seguro a goal
      PREPARAR contenedores para feedback y result
      INICIALIZAR secuencia con [0, 1]
      PUBLICAR "Starting/Executing" en logs

      SI order == 0 → sequence = [0]
      SI order == 1 → sequence = [0, 1]

      PARA i desde 2 hasta order:
        SI se solicitó cancelación →
          MARCAR meta como CANCELADA y SALIR
        CALCULAR siguiente = sequence[i-1] + sequence[i-2]
        AÑADIR a sequence
        PUBLICAR feedback.partial_sequence = sequence
        DORMIR 1 segundo (simular trabajo)

      MARCAR meta como EXITOSA con result.sequence = sequence

EJECUTAR rclcpp::spin hasta cerrar
```

**Evidencia en la terminal:**  
Al ejecutar el nodo y enviar una meta `order: 10`, se observó:
- `Received goal request with order: 10`
- Publicación de *feedback* cada ~1 s
- `Goal Succeeded`
- `Result.sequence` con 11 términos (0..55)

---

### 2) `CMakeLists.txt` (integración del ejecutable)

Cambios típicos realizados para compilar el servidor de acciones en C++:

1. **Dependencias** en `find_package(...)`:
   - `rclcpp`
   - `rclcpp_action`
   - `arduinobot_msgs` (donde está la acción `Fibonacci`)

2. **Ejecutable** y *target*:
   - `add_executable(simple_action_server_node src/simple_action_server.cpp)`
   - `ament_target_dependencies(simple_action_server_node rclcpp rclcpp_action arduinobot_msgs)`

3. **Instalación** para poder usar `ros2 run`:
   - `install(TARGETS simple_action_server_node DESTINATION lib/${PROJECT_NAME})`

Con estos cambios, `colcon build` genera el binario y `ros2 run arduinobot_cpp_examples simple_action_server_node` lo ejecuta correctamente.

---

### 3) `package.xml` (metadatos y dependencias)

Se declararon/confirmaron las dependencias necesarias para compilar y ejecutar el nodo:

- `<buildtool_depend>ament_cmake</buildtool_depend>`  
- `<depend>rclcpp</depend>`  
- `<depend>rclcpp_action</depend>`  
- `<depend>arduinobot_msgs</depend>`  

También se mantienen los metadatos del paquete (nombre, versión, licencia, autor, mantenedor).

---

## 🖥️ Comandos y flujo de trabajo (dos terminales)

A continuación se explican **uso, propósito y función** de cada comando visto en la clase. Separamos por terminal para conservar el flujo.

### 🧵 Terminal 1 (compilación y ejecución del servidor)

1. **`colcon build`**  
   - **Función:** compila todos los paquetes del *workspace*.  
   - **Propósito:** generar los binarios y mensajes (incluidas acciones) para poder ejecutar nodos.  
   - **Salida observada:** todos los paquetes compilan; `arduinobot_cpp_examples` muestra *warnings* pero finaliza OK.

2. **`. install/setup.bash`** *(sourcing)*  
   - **Función:** exporta al entorno las rutas de instalación (binarios, mensajes, recursos).  
   - **Propósito:** que `ros2` y el *shell* encuentren los ejecutables recién compilados.  
   - **Nota:** debe ejecutarse **en cada nueva terminal** tras compilar.

3. **`ros2 run arduinobot_cpp_examples`** *(sin segundo argumento)*  
   - **Función:** lista los ejecutables disponibles dentro del paquete.  
   - **Útil para:** confirmar el nombre correcto del ejecutable (`simple_action_server_node`).

4. **`ros2 run arduinobot_cpp_examples simple_action_server_node`**  
   - **Función:** ejecuta el servidor de acciones.  
   - **Observado:** logs de inicio, recepción de meta, *feedback* periódico y finalización exitosa.

### 🧵 Terminal 2 (inspección y prueba con la CLI de acciones)

1. **`. install/setup.bash`**  
   - Igual que en la Terminal 1: habilita el entorno para usar `ros2` con los artefactos del *workspace*.

2. **`ros2 action list`**  
   - **Función:** lista los *topics* de acciones disponibles.  
   - **Observado:** `/fibonacci` (expuesto por el servidor ejecutándose en la Terminal 1).

3. **`ros2 action info /fibonacci -t`**  
   - **Función:** muestra detalles del *topic* de acción.  
   - **Propósito:** verificar cuántos **clientes** y **servidores** están conectados, y el **tipo** de la acción.  
   - **Observado:** 1 servidor (`/simple_action_server`), 0 clientes.

4. **`ros2 action send_goal /fibonacci arduinobot_msgs/action/Fibonacci "order: 10" -f`**  
   - **Función:** envía una **meta** al servidor con el mensaje especificado.  
   - **Parámetros relevantes:**
     - `arduinobot_msgs/action/Fibonacci` → tipo de acción.  
     - `"order: 10"` → contenido del *goal* (YAML).  
     - `-f` / `--feedback` → imprime la retroalimentación mientras se ejecuta.  
   - **Resultado esperado:** *feedback* incremental con la secuencia parcial y, al final, la `Result.sequence` completa.

> **Errores comunes evitados:**  
> - Asegúrate de escribir el tipo de acción y el YAML correctamente (comillas incluidas).  
> - Ejecuta el *sourcing* antes de usar `ros2`.  
> - Mantén el servidor corriendo en otra terminal al enviar la meta.

---

## ✅ Evidencia de ejecución (resumen)

- **Servidor activo:** `Action servers: 1 → /simple_action_server [arduinobot_msgs/action/Fibonacci]`  
- **Secuencia generada para `order: 10`:** `0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55`  
- **Logs del servidor:** *Publish Feedback* ~cada 1 s, seguido de *Goal Succeeded*.

---

## 🛠️ Solución a *warnings* (opcional)

- **Parámetros sin usar:**  
  Dentro de `goalCallback(...)` y `cancelCallback(...)`, puedes añadir una línea al inicio:  
  ```cpp
  (void)uuid;       // en goalCallback
  (void)goal_handle; // en cancelCallback
  ```
  o marcar los parámetros como `[[maybe_unused]]`.

> No es obligatorio; la compilación ya fue **exitosa** y los *warnings* son informativos.

---

## 🧪 Reproducción rápida

```bash
# Terminal 1
colcon build
. install/setup.bash
ros2 run arduinobot_cpp_examples simple_action_server_node

# Terminal 2
. install/setup.bash
ros2 action list
ros2 action info /fibonacci -t
ros2 action send_goal /fibonacci arduinobot_msgs/action/Fibonacci "order: 10" -f
```

---

### Última actualización
Generado automáticamente el 2025-09-18 22:01:42.
