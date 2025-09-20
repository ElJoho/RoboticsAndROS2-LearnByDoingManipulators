# Clase 80 — `simple_action_client.py` (ROS 2 Actions en Python)

En esta clase implementamos y probamos un **Action Client** en Python que se comunica con el **Action Server** `Fibonacci` desarrollado en la clase anterior. También actualizamos `setup.py` para poder ejecutar el *client* con `ros2 run`, y validamos la interacción completa **servidor ↔ cliente** viendo *feedback* y *resultados* en la terminal.

---

## Archivos trabajados
- `arduinobot_py_examples/simple_action_client.py` — Nodo **cliente** de acciones que envía una meta (goal) al servidor `fibonacci` y procesa *feedback* y *resultados*.
- `arduinobot_py_examples/setup.py` — Registro de *entry points* (`console_scripts`) para lanzar el cliente con `ros2 run`.

---

## `simple_action_client.py` — ¿Qué hace?
Implementa un nodo ROS 2 (`SimpleActionClient`) que:
1) Espera a que exista un *Action Server* llamado `fibonacci` con la interfaz `Fibonacci`.
2) Envía una meta (`order=10`) para que el servidor calcule la **secuencia de Fibonacci**.
3) Muestra **feedback** periódico con la secuencia parcial.
4) Muestra el **resultado final** (secuencia completa) y apaga ROS 2.

### Pseudocódigo del script
```text
iniciar nodo ROS 2 con nombre "simle_action_client"  # (nombre tal como aparece en el código)
crear ActionClient(Fibonacci, nombre_accion="fibonacci")

esperar a que el servidor esté disponible (wait_for_server)

crear mensaje de meta: goal = Fibonacci.Goal()
goal.order = 10

future_goal = send_goal_async(goal, feedback_callback=feedbackCallback)

future_goal.add_done_callback(responseCallback)

función responseCallback(future):
    goal_handle = future.result()
    si goal_handle.accepted == False:
        log "Goal Rejected" y terminar
    si no:
        log "Goal Accepted"
        future_result = goal_handle.get_result_async()
        future_result.add_done_callback(resultCallback)

función resultCallback(future):
    result = future.result().result
    log "Result: <secuencia completa>"
    rclpy.shutdown()  # terminar el nodo

función feedbackCallback(feedback_msg):
    log "Received Feedback: <secuencia parcial>"

main():
    rclpy.init()
    instanciar SimpleActionClient
    rclpy.spin()  # mantener el nodo corriendo
```

> Nota: En el código el nombre del nodo es `"simle_action_client"` (con una pequeña errata), y los mensajes de *log* se imprimen tal cual están definidos. Esto no afecta la ejecución.

---

## `setup.py` — Cambios y propósito
- **`entry_points.console_scripts`**: Se añadió/confirmó la entrada
  ```ini
  simple_action_client = arduinobot_py_examples.simple_action_client:main
  ```
  Esto permite lanzar el cliente con:
  ```bash
  ros2 run arduinobot_py_examples simple_action_client
  ```
- El resto de campos (`packages`, `data_files`, `install_requires`, etc.) permanecen según la plantilla del paquete.

**¿Por qué es necesario?**  
Sin registrar el *entry point*, `ros2 run` no sabría qué función `main()` ejecutar dentro del paquete. Con esta línea, `ament` instala un ejecutable “ligero” que invoca `simple_action_client:main` durante el *build*.

---

## Ejecución en terminales y salidas observadas

### Terminal 1 — Compilación y lanzamiento del **Action Server**
Comandos:
```bash
colcon build
. install/setup.bash
ros2 run arduinobot_py_examples simple_action_server
```
Salida (resumen):
```
Starting the server
Received goal request with order 10
Feedback: [0, 1, 1]
Feedback: [0, 1, 1, 2]
...
Feedback: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```
**Qué demuestra:** el servidor arranca, acepta la meta con `order=10` y publica *feedback* con la secuencia parcial hasta completar la serie.

### Terminal 2 — Lanzamiento del **Action Client** en Python
Comandos:
```bash
. install/setup.bash
ros2 run arduinobot_py_examples simple_action_client
```
Salida (resumen):
```
[simle_action_client]: Goal Accepted
[simle_action_client]: Received Feedback: [0, 1, 1]
[simle_action_client]: Received Feedback: [0, 1, 1, 2]
...
[simle_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```
**Qué demuestra:** el cliente se conecta al servidor, confirma **Goal Accepted**, recibe *feedback* incremental y finalmente el **Result** con la secuencia completa; luego cierra (`rclpy.shutdown()`).

---

## Pasos completos (receta rápida)
1. **Compilar** el workspace:
   ```bash
   colcon build
   ```
2. **Preparar entorno** (en cada terminal):
   ```bash
   . install/setup.bash
   ```
3. **Lanzar servidor** (Terminal 1):
   ```bash
   ros2 run arduinobot_py_examples simple_action_server
   ```
4. **Lanzar cliente** (Terminal 2):
   ```bash
   ros2 run arduinobot_py_examples simple_action_client
   ```

Si el servidor no está activo, el cliente se **bloquea** en `wait_for_server()` hasta que aparezca el servidor `fibonacci`.

---

## Resultados de aprendizaje
- Crear y usar un **ActionClient** en Python (`rclpy.action.ActionClient`).
- Gestionar flujos **asíncronos** con *futures* y *callbacks* (`send_goal_async`, `add_done_callback`, `get_result_async`).
- Registrar ejecutables Python en `setup.py` mediante **`console_scripts`**.