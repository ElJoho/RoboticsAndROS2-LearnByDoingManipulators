# Clase 44 — `gazebo.launch.py`

Notas de clase y guía rápida para simular el **Arduinobot** en Gazebo (ROS 2 Humble)

---

## 🎯 Objetivo de la clase

En esta lección cerramos la construcción del *gemelo digital* del robot para poder **visualizarlo y simularlo en Gazebo**, reutilizando lo ya hecho para `display.launch.py` y añadiendo:

* uso de **tiempo simulado**,
* configuración de la **ruta de recursos** para que Gazebo encuentre URDF/meshes,
* el **arranque de Gazebo** con un mundo vacío,
* el ***spawn*** del robot leyendo `/robot_description`,
* y el **bridge** Gazebo ↔ ROS 2 para exponer `/clock`.
  (Ver `gazebo.launch.py` y `package.xml` en este repo.) &#x20;

---

## 📦 Archivos relevantes

* **`launch/gazebo.launch.py`** — Launch file que:

  1. publica el URDF con `robot_state_publisher` y `use_sim_time=True`,
  2. exporta `GZ_SIM_RESOURCE_PATH`,
  3. **incluye** el launch de Gazebo (`ros_gz_sim`),
  4. hace *spawn* del robot leyendo `/robot_description`,
  5. y crea el **bridge** de `/clock`.&#x20;

* **`package.xml`** — Declara dependencias de ejecución, incluyendo `ros_gz_sim` y `ros_gz_bridge` necesarias para simular y puentear mensajes entre Gazebo y ROS 2.&#x20;

---

## 🧠 Conceptos clave (resumen de la TRANSCRIPTION)

* El launch file define una única función `generate_launch_description()` que retorna una `LaunchDescription` con **todas las acciones** (nodos/ajustes) a ejecutar.
* Reutilizamos el bloque que ya publicaba el `robot_description` desde el Xacro y **añadimos `use_sim_time=True`** para que los nodos usen el reloj simulado.
* Se configura `GZ_SIM_RESOURCE_PATH` para que Gazebo localice **URDF/meshes**.
* Se **incluye** el launch de `ros_gz_sim` y se pasa `gz_args="-v 4 -r empty.sdf"` para abrir un mundo vacío con logs detallados y simulación corriendo.
* Se **spawnea** el robot desde `/robot_description` con `ros_gz_sim create`.
* Se levanta el **bridge** `ros_gz_bridge parameter_bridge` para publicar `/clock` (tipo `rosgraph_msgs/msg/Clock`) mapeado desde `gz.msgs.Clock`.
* Finalmente, se ordena todo en la `LaunchDescription` y se guarda el archivo.

---

## 🧩 ¿Cómo funciona `gazebo.launch.py`? (paso a paso)

1. **Imports**: Se importan clases de `launch`/`launch_ros` para describir acciones, nodos, parámetros y sustituciones.&#x20;
2. **Rutas del paquete**: Se obtiene `arduinobot_description_dir` con `get_package_share_directory("arduinobot_description")`.&#x20;
3. **Entorno de Gazebo**: Se define `GZ_SIM_RESOURCE_PATH` para que Gazebo encuentre los recursos del robot (URDF/meshes).&#x20;
4. **Argumento `model`**: Se declara un argumento de launch para apuntar al Xacro por defecto `urdf/arduinobot.urdf.xacro`.&#x20;
5. **`robot_description`**: Se convierte el Xacro a URDF en tiempo de lanzamiento con `Command(["xacro ", LaunchConfiguration("model")])`.&#x20;
6. **Publicación del URDF + reloj simulado**: Se lanza `robot_state_publisher` con `use_sim_time=True`.&#x20;
7. **Arranque de Gazebo**: Se **incluye** el launch `gz_sim.launch.py` de `ros_gz_sim` con `gz_args="-v 4 -r empty.sdf"`.&#x20;
8. **Spawn del robot**: Se ejecuta `ros_gz_sim create` leyendo desde `-topic robot_description` y nombrando la entidad `-name arduinobot`.&#x20;
9. **Bridge `/clock`**: Se ejecuta `ros_gz_bridge parameter_bridge` con el mapeo `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`.&#x20;
10. **LaunchDescription**: Se devuelven las acciones en orden lógico para que el ecosistema quede listo.&#x20;

---

## 🧾 Pseudocódigo del flujo de `gazebo.launch.py`

```pseudocode
function generate_launch_description():
    # 1) Resolver rutas del paquete
    pkg_share = get_share_dir("arduinobot_description")

    # 2) Exportar ruta de recursos para Gazebo
    set_env("GZ_SIM_RESOURCE_PATH", resolve(parent_of(pkg_share)))

    # 3) Declarar argumento 'model' -> Xacro por defecto
    model_arg = declare_arg("model", path_join(pkg_share, "urdf", "arduinobot.urdf.xacro"))

    # 4) Construir robot_description ejecutando xacro en tiempo de launch
    robot_description = command_output("xacro ", get_arg("model"))

    # 5) Publicar URDF con tiempo simulado
    node_rsp = run_node("robot_state_publisher",
                        params = {"robot_description": robot_description,
                                  "use_sim_time": True})

    # 6) Incluir el launch de Gazebo con argumentos (mundo vacío, verbose, run)
    gazebo = include_launch(share_of("ros_gz_sim")/launch/"gz_sim.launch.py",
                            args = {"gz_args": "-v 4 -r empty.sdf"})

    # 7) Spawnear el robot leyendo /robot_description
    spawn = run_node("ros_gz_sim", "create",
                     args = ["-topic", "robot_description",
                             "-name", "arduinobot"])

    # 8) Bridge del reloj Gazebo -> ROS 2
    bridge = run_node("ros_gz_bridge", "parameter_bridge",
                      args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])

    # 9) Devolver el plan de lanzamiento
    return LaunchDescription([model_arg, set_env, node_rsp, gazebo, spawn, bridge])
```

---

## ⚙️ Dependencias declaradas en `package.xml`

El paquete **declara** como dependencias de ejecución, entre otras:
`robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `urdf`, `xacro`, `rviz2`, `joint_state_publisher_gui`, `ros2launch`.
Asegúrate de tenerlas instaladas en tu sistema antes de compilar.&#x20;

---

## ▶️ Compilar y ejecutar

```bash
# 1) En la raíz del workspace
colcon build --symlink-install

# 2) Cargar el overlay
source install/setup.bash

# 3) Lanzar Gazebo con el Arduinobot
ros2 launch arduinobot_description gazebo.launch.py

# (Opcional) Si quieres especificar otro Xacro/URDF:
ros2 launch arduinobot_description gazebo.launch.py model:=/ruta/a/tu/robot.urdf.xacro
```

---

## 🛠️ Consejos y solución de problemas

* **No aparece el robot / fallan los meshes**
  Verifica que `GZ_SIM_RESOURCE_PATH` apunte a un directorio que contenga tus **URDF/meshes**. El launch exporta el **padre** del `share` del paquete; si tus recursos están dentro del `share` del paquete, puedes ajustar esa ruta según tu estructura.&#x20;

* **`xacro` no encontrado**
  Asegúrate de tener `xacro` instalado y en el `PATH` (está declarado como `exec_depend` en `package.xml`).&#x20;

* **El mundo no abre o Gazebo no arranca**
  Revisa que `ros_gz_sim` esté instalado (aparece en `package.xml`) y que el launch `gz_sim.launch.py` exista en el share de ese paquete.&#x20;

* **El tiempo simulado no avanza en tus nodos**
  Comprueba que tus nodos lean `/clock` y que tengas `use_sim_time` en `True` (en este launch file ya se configura para `robot_state_publisher`).&#x20;
