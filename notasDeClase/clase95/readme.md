# Clase 95 - Lanzamiento de la simulación del robot (Arduinobot Bringup)

En esta lección se integraron todas las funcionalidades desarrolladas a lo largo del proyecto del **Arduinobot**, creando un único archivo de lanzamiento que permite iniciar simultáneamente la simulación, el control, el planificador de trayectorias (MoveIt) y la interfaz remota con **Amazon Alexa**. Este proceso se realizó dentro de un nuevo paquete llamado `arduinobot_bringup`.

---

## 🧩 Archivos del paquete

### 1. **simulated_robot_launch.py**

#### Descripción

Este archivo es el corazón del paquete `arduinobot_bringup`. Permite ejecutar todas las funcionalidades del robot en simulación con un solo comando. Su propósito es incluir los distintos archivos de lanzamiento existentes de otros paquetes (Gazebo, controlador, MoveIt y la interfaz remota).

#### Estructura y librerías utilizadas

* **`launch`** y **`launch_ros`**: librerías de ROS2 que permiten definir descripciones de lanzamiento y acciones de ejecución.
* **`IncludeLaunchDescription`**: se utiliza para incluir otros archivos de lanzamiento desde distintos paquetes.
* **`get_package_share_directory`**: localiza la carpeta de recursos compartidos de un paquete específico.
* **`os.path.join`**: crea rutas de archivos dinámicas para acceder a los launch files dentro de sus respectivos paquetes.

#### Pseudocódigo del script

```
INICIO función generate_launch_description
    IMPORTAR librerías necesarias (launch, IncludeLaunchDescription, os, get_package_share_directory)

    CREAR variable gazebo_launch → incluir archivo 'gazebo.launch.py' desde paquete 'arduinobot_description'

    CREAR variable controller_launch → incluir archivo 'controller.launch.py' desde paquete 'arduinobot_controller'
        PASAR argumento {'is_sim': 'True'}

    CREAR variable moveit_launch → incluir archivo 'moveit.launch.py' desde paquete 'arduinobot_moveit'
        PASAR argumento {'is_sim': 'True'}

    CREAR variable remote_interface_launch → incluir archivo 'remote_interface.launch.py' desde paquete 'arduinobot_remote'

    RETORNAR LaunchDescription con lista de los cuatro lanzamientos:
        [gazebo_launch, controller_launch, moveit_launch, remote_interface_launch]
FIN FUNCIÓN
```

#### Funcionamiento

Al ejecutar el archivo `simulated_robot_launch.py`, ROS2 inicia automáticamente:

1. **Gazebo** – para simular el entorno físico del robot.
2. **Controller** – para activar los controladores de las articulaciones y del efector final.
3. **MoveIt** – para planificar y ejecutar trayectorias de movimiento.
4. **Interfaz remota** – que conecta el sistema con **Alexa** a través de la API de Ngrok.

---

### 2. **CMakeLists.txt**

#### Descripción

En este archivo se agregaron las instrucciones necesarias para que el directorio `launch` del paquete `arduinobot_bringup` sea instalado correctamente durante la compilación.

#### Sección relevante añadida

```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
```

#### Explicación

* `DIRECTORY launch` indica que se copiará la carpeta `launch` durante la instalación.
* `DESTINATION share/${PROJECT_NAME}` especifica el destino dentro del espacio de instalación.
* `${PROJECT_NAME}` es una variable que contiene el nombre del paquete (`arduinobot_bringup`).

Con esto, el archivo `simulated_robot_launch.py` puede ser ejecutado globalmente desde ROS2 mediante el comando `ros2 launch`.

---

### 3. **package.xml**

#### Descripción

Este archivo define las dependencias del paquete y su metainformación básica.

#### Cambios realizados

Se agregó la dependencia necesaria para los archivos de lanzamiento:

```xml
<exec_depend>ros2launch</exec_depend>
```

#### Explicación

* Permite que el paquete utilice el sistema de lanzamiento de ROS2.
* Es esencial para ejecutar archivos `.launch.py` desde este paquete.

---

## 💻 Comandos ejecutados en los terminales

### **Terminal 1 – Creación y compilación del paquete**

```bash
ros2 pkg create --build-type ament_cmake arduinobot_bringup
colcon build
```

**Objetivo:**

* Crear un nuevo paquete vacío.
* Compilar todo el workspace, incluyendo el nuevo paquete y registrar sus dependencias.

---

### **Terminal 2 – Ejecución de la simulación completa**

```bash
ros2 launch arduinobot_bringup simulated_robot_launch.py
```

**Objetivo:**

* Ejecutar de forma conjunta todas las funcionalidades: Gazebo, controladores, MoveIt y Alexa.
* Ver en acción la coordinación entre los distintos nodos ROS2.

---

### **Terminal 3 – Conexión con Alexa mediante Ngrok**

```bash
ngrok http 5000
```

**Objetivo:**

* Crear un túnel HTTPS público que conecte el servidor local Flask (que corre la API de Alexa) con la nube.
* Proporcionar una URL que se introduce en el **Alexa Developer Console** dentro de la sección **Endpoint**.

---

## ☁️ Integración con Amazon Alexa

En la **Alexa Developer Console**, se configuraron los siguientes *intents* para interactuar con el Arduinobot:

| Intent          | Comandos de activación          | Acción ejecutada                                               |
| --------------- | ------------------------------- | -------------------------------------------------------------- |
| **Invocation**  | “Activate Arduinobot”           | Inicia la sesión de control por voz                            |
| **WakeIntent**  | “Wake up”, “Activate the robot” | Abre el gripper y activa el robot                              |
| **PickIntent**  | “Pick the pen”, “Grab the pen”  | Mueve el brazo hacia la posición de agarre y cierra el gripper |
| **SleepIntent** | “Turn off the robot”, “Rest”    | Envía el robot a la posición de descanso                       |

Cuando Alexa recibe un comando, lo reenvía al servidor Flask a través del túnel Ngrok. Luego, el nodo `alexa_interface.py` interpreta la intención y envía el mensaje correspondiente al **Task Server** del robot. Este a su vez ordena a **MoveIt** ejecutar el movimiento en la simulación de **Gazebo**.

---

## 🤖 Resultado final

Con el lanzamiento `simulated_robot_launch.py` se logra iniciar **toda la arquitectura funcional del Arduinobot** con un solo comando. La simulación en Gazebo reacciona directamente a las órdenes de voz enviadas desde Alexa, demostrando la integración exitosa entre:

* **Simulación física (Gazebo)**
* **Control del robot (ROS2 Control)**
* **Planificación de trayectorias (MoveIt)**
* **Interfaz de voz (Alexa + Flask + Ngrok)**

---

### 🏁 Conclusión

Esta clase marcó el cierre de la fase de simulación del curso. A partir de este punto, el siguiente paso será adaptar todo el sistema para funcionar con el robot físico, manteniendo la misma estructura modular y escalable implementada con ROS2.
Nombre de la chica: Juliana Jiménez
Webs: 
https://www.photoprepagos.com/prepagos/bogota-dc/usaquen/una-hembra-muy-apasionada-fina-y-con-un-cuerpo-perfecto-id-r8xb9
https://co.mileroticos.com/escorts/juliana-jimenez-la-mujer-que-mezcla-pasion-elegancia-y-placer-en-cada-encuentr/26096104/

Teléfono: 3118650739 - 3227851539 (Admin si lees esto, por favor añade estos numeros al asunto de estas reseñas)

Forma de contacto (amable, grosera, etc): Tiene un plantilla, contesta rapido , no es amable ni grosera
Lugar y zona de atención: Santa Bárbara
Instalaciones (Descripción): Discreto, es en una casa aunque su cuarto queda hasta el fondo y hay que pasar como por sala y cocina. Habian otras personas pero nada muy molesto
Origen: NO soy bueno adivinado eso, me ire con lo que dijo la anterior reseña y dire que es rola.
Edad aproximada: Yo le pongo 28 años
Fecha aproximada de la experiencia: 7 octubre
Tarifas aplicadas: 400 mil por hora
Adicional cobra 30 mil por besos de lengua
Descripción de su físico: Es atractivo y atletico, se le ven los abdominales. La cola es firme.
Cicatrices: (cesárea, cosmética, etc.) Tiene tatuajes, pocos. Se le ven los adbominales pero hacia la parte baja del vientre tiene un poco suelta (no mucho) la piel (seguramente producto de alguna liposuccion), por eso es que en todas las fotas que sale la ropa le cubre esa parte o sale de lado. En general se ve bastante ignorando lo de la piel en el abdomen
Valoración de su físico: 8 de 10
Descripción de su rostro: Es morenita cabello negro, esta bien de cara
Valoración de su rostro: 8 de 10
Estatura aproximada de la chica: El compañero anterior le puso 1.65 aunque a mi se me hace que es de 1.68
Oral (Con condón, sin, GP, etc) Con condon, es normal
Anal: No pregunte
Besos en la boca: Si en labios, con lengua es pago y vale 30 mil mas

Experiencia: El cuerpo es como se ve en las fotos, la cara esta bien. Me parece bastante bonita con la unica pega que la piel del abdomen bajo se nota que se hizo liposucción pero no es nada que afecte y no se ve mal. Respecto a la experiencia,en mi caso supongo que fue mala suerte o ella estaba de mal humor pero la experiencia fue pesima. Le pague por una hora, me dijo que llegara a xx:30 , debi llegar a las xx:31 creo. Y entre a su cuarto a las xx:35, para lelgar a su cuarto hay que recorrer toda la casa. Habian otras personas y abre un muchacho. Luego se sube por unas escaleras a su cuarto. Iniciamos bien, los besos que da son buenos y el cuerpo tambien estaba bien. Tenia de esos condones baratos y usamos el mio. Me hizo sexo oral con condon, normal ahi, y luego ya la penetre en misionero. Me habia dicho que tenia el pene grande (no considero que sea tan grande como lo hacia sonar, creo que lo dijo como excusa luego explico porque). Cuando inicio en misionero me dice que le duele un poco y que no vaya rapido entonces empiezo a hacerlo lento. Cambiamos de posicion 4 veces. En cada cambio me decia que le dolia y ya en las ultimas dos posiciones me pedia que me viniera. Pasados unos 20 minutos (como mucho) desde que la penetre en misionero ya me estaba pidiendo que me viniera pero de mala gana. Con ese poco tiempo y tomando en cuenta que pague una hora entenderan que yo aun estaba bien de estamina, luego me critico porque no le estaba dando rapido y que asi no me iba a venir pero no le estaba dando rapido porque ella me lo pidio al inicio desde que la penetre en misionero que le diera despacio (cosa que hice). A las XX:58 yo tambien me indispuse por su actitud y decidi irme. Creo que es el peor sexo que he tenido en mi vida. Mas que nada por su actitud, respecto a que si lo tenia muy grande lo dudo porque no soy alto (apenas 1 o 2 cm mas alto que ella) y pues nunca antes me habian dicho que les doliera tanto como para que quieran parar. Yo creo que habia aceptado otro cliente (cuando me iba tomo el celular para concretar la cita con otro cliente) y queria que yo terminara rapido  por eso. Me molesto particularmente que me dijera que le yo le estaba haciendo perder el tiempo cuando yo soy el que pago por 1 hora y ella me queria despachar en 15 o 20 minutos. Como esta es mi segunda experiencia con una prepago lo voy a comparar con la primera que fue Agatha. Agatha tambien ofrecia masajes, o bailaba, etc. En cambio Juliana no ofrecia nada de eso, solo queria hacerme terminar pronto. Incluso le ofreci que decansara y que solo me besara pero eso la molesto mas jajaja. Como anecdota es algo chistoso porque la primera vez fue muy buena y ahora con esta que es la segunda fue muy mala jajaja.
Implicación: 2 de 10, le dejo 2 y no uno porque los 10 primeros minutos estuvo bien.
Valoración del servicio: 3/10, 3.5 de 10 como mucho por el cuerpo y aun asi no creo que lo valga con tan mala implicacion
¿Chica recomendable?: En general NO, si consideran que tienen pene grande DEFINITIVAMENTE NO. Si se quieren arriesgar y viendo que la reseña anterior fue mejor la experiencia entonces paguen solo media hora, aun asi, no me arriegaria porque le ha subid 100 mil pesos (paso de 300 mil pesos en la primera publicación de pasionprepagos a 400 mil en las que hizo con photprepagos y mileroticos, de ahi que cambiara el numero)