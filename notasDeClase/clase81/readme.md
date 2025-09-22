# Clase 81 — Cliente de Acciones en C++ (`simple_action_client.cpp`)

En esta clase implementamos un **nodo cliente de acciones en C++** que se conecta al **servidor de la acción Fibonacci** desarrollado previamente. Se valida la disponibilidad del servidor, se envía una meta (`order=10`), se procesa la **retroalimentación** (secuencia parcial) y el **resultado** (secuencia completa), y se finaliza el nodo.

---

## 1) `simple_action_client.cpp` — Funcionamiento (pseudocódigo)

```pseudocode
INCLUDES: rclcpp, rclcpp_action, arduinobot_msgs/action/Fibonacci, rclcpp_components, sstream

NAMESPACE arduinobot_cpp_examples:
  CLASE SimpleActionClient hereda de rclcpp::Node
    CONSTRUCTOR(options: NodeOptions const&)
      → Node("simple_action_client", options)
      client_ ← create_client<Fibonacci>(this, "fibonacci")
      timer_  ← create_wall_timer(1s, bind(timerCallback))
    
    PRIVADO:
      client_: SharedPtr<Client<Fibonacci>>
      timer_:  SharedPtr<TimerBase>

      FUNC timerCallback():
        timer_.cancel()               # Ejecutar una sola vez
        SI !client_.wait_for_action_server():
          LOG ERROR "Action server not available after waiting"
          rclcpp::shutdown()
          RETURN

        goal_msg ← Fibonacci::Goal{ order = 10 }
        LOG INFO "Sending Goal"

        send_goal_options ← Client<Fibonacci>::SendGoalOptions()
        send_goal_options.goal_response_callback ← bind(goalCallback, _1)
        send_goal_options.feedback_callback      ← bind(feedbackCallback, _1, _2)
        send_goal_options.result_callback        ← bind(resultCallback, _1)

        client_.async_send_goal(goal_msg, send_goal_options)

      FUNC goalCallback(goal_handle: SharedPtr<ClientGoalHandle<Fibonacci>> const&):
        SI !goal_handle:
          LOG ERROR "Goal was rejected by the server"
        SINO:
          LOG INFO "Goal Accepted by the Server, waiting for result"

      FUNC feedbackCallback(handle: SharedPtr<ClientGoalHandle<Fibonacci>>,
                            feedback: shared_ptr<const Fibonacci::Feedback>):
        ss ← "Next number in sequence received: "
        PARA cada number EN feedback.partial_sequence:
          ss += f"{number} "
        LOG INFO ss

      FUNC resultCallback(result: ClientGoalHandle<Fibonacci>::WrappedResult const&):
        SWITCH result.code:
          CASO SUCCEEDED: (continuar para imprimir resultado)
          CASO ABORTED:  LOG ERROR "Goal was aborted";  RETURN
          CASO CANCELED: LOG ERROR "Goal was canceled"; RETURN
          DEFAULT:       LOG ERROR "Unknown result code"; RETURN

        ss ← "Next number in sequence received: "
        PARA cada number EN result.result.sequence:
          ss += f"{number} "
        LOG INFO ss
        rclcpp::shutdown()
```

**Notas clave**
- Se usa un **timer de 1 s** para diferir el envío de la meta y garantizar que el nodo se inicialice por completo.
- `wait_for_action_server()` evita enviar metas si el servidor aún no está disponible.
- Se configuran tres *callbacks*: **goal_response**, **feedback** y **result**.
- Al finalizar con éxito, se imprime la secuencia completa y se ejecuta `rclcpp::shutdown()`.

---

## 2) Cambios en `CMakeLists.txt` (registro de componente y dependencias)

En el paquete `arduinobot_cpp_examples` se agrega el **objetivo tipo biblioteca compartida** para el cliente de acciones y se registra como **componente** con su ejecutable:

- **Definición de la librería del cliente**:  
  `add_library(simple_action_client SHARED src/simple_action_client.cpp)`
- **Dependencias** con `ament_target_dependencies`:  
  `arduinobot_msgs`, `rclcpp`, `rclcpp_action`, `rclcpp_components`.
- **Registro del nodo como componente** y creación del ejecutable asociado:  
  `rclcpp_components_register_node(simple_action_client
   PLUGIN "arduinobot_cpp_examples::SimpleActionClient"
   EXECUTABLE simple_action_client_node)`
- **Instalación** de los binarios/librerías del servidor y cliente de acciones.

Estos fragmentos permiten que `ros2 run arduinobot_cpp_examples simple_action_client_node` funcione y que el nodo también pueda cargarse como componente. fileciteturn0file0

---

## 3) Comandos usados en ambas terminales

### Terminal 1 — Servidor de la acción
```bash
cd ~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws/
colcon build                      # Compilar todo el workspace
. install/setup.bash              # Cargar el overlay del workspace
ros2 run arduinobot_cpp_examples simple_action_server_node  # Iniciar servidor Fibonacci
```
**Qué se observa**  
- Mensajes tipo: *“Starting the Action Server”*, luego *“Received goal request with order: 10”*, publicaciones de *“Publish Feedback”* y finalmente *“Goal Succeeded”* (cuando el cliente envía la meta y el servidor termina).  
- Pueden aparecer **warnings** por parámetros no usados en `simple_action_server.cpp`; no impiden la ejecución.

### Terminal 2 — Cliente de la acción
```bash
cd ~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws/
. install/setup.bash
ros2 run arduinobot_cpp_examples simple_action_client_node  # Iniciar cliente Fibonacci
```
**Qué se observa**  
- `Sending Goal` → el cliente envía `order=10`.  
- `Goal Accepted by the Server, waiting for result` → el servidor acepta la meta.  
- Varias líneas `Next number in sequence received: ...` con la **secuencia parcial** (feedback).  
- Al completar el cálculo, se imprime la **secuencia completa** y el cliente hace `shutdown`.

> Para detener el servidor manualmente: `Ctrl + C` en la terminal del servidor.

---

## 4) Requisitos previos y tips
- Haber compilado sin errores con `colcon build` y **sourciar** siempre `install/setup.bash` en cada terminal.
- Si el cliente no arranca o no avanza, verifique que el **servidor esté en ejecución**. El cliente espera al servidor con `wait_for_action_server()`.
- Si aparecen *warnings* de parámetros no usados, puede silenciarlos con `[[maybe_unused]]` o haciendo `(void)param;` dentro de la función (opcional).

---

## 5) Resultado esperado (resumen)
- **Servidor (T1)**: recibe la meta `order=10`, publica *feedback* periódico y finaliza con **Goal Succeeded**.  
- **Cliente (T2)**: muestra *feedback* incremental de la secuencia de Fibonacci y al final imprime la **secuencia completa**, luego termina.