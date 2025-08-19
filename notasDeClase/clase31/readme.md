# Clase 32 .URDF en ROS2

## Introducción
Cuando se crea un robot, ya sea real o simulado, el primer paso es **definir su estructura**. Esto implica modelar cada componente, especificar qué partes son fijas, cuáles son móviles y dónde se colocan los sensores.  

En ROS2 se utiliza **URDF (Unified Robot Description Format)**, un estándar de la comunidad robótica basado en **XML**, para describir la estructura de un robot y las propiedades de sus componentes.

---

## Estructura básica de un archivo URDF
Un archivo URDF está contenido en un bloque principal:

- `<robot>`: Define la estructura general del robot y contiene todos los elementos.

Dentro de este bloque se definen principalmente dos tipos de elementos:

### 1. `<link>`
Representa cualquier componente del robot.  
- Define un **marco de referencia** con propiedades asociadas.  
- Puede incluir:
  - `<visual>`: Malla o modelo 3D para visualización.  
  - `<collision>`: Geometría utilizada para detección de colisiones.  
  - `<inertial>`: Propiedades físicas como masa, volumen e inercia, necesarias para la simulación física.

### 2. `<joint>`
Conecta dos enlaces y define el tipo de relación entre ellos.  
- Propiedades principales:
  - **parent**: enlace padre.  
  - **child**: enlace hijo.  
  - **type**: tipo de unión (ej. `fixed`, `revolute`, `prismatic`).  
  - **axis**: eje de movimiento o rotación.  
  - **limits**: restricciones en el movimiento relativo entre enlaces.  

Cada enlace puede tener **un solo padre**, pero un padre puede tener **múltiples hijos**. Esta estructura se asemeja a un **árbol jerárquico**.

---

## Importancia del URDF
- Permite **visualizar** cómo los componentes del robot están conectados.  
- Proporciona las bases para la **simulación física**, ya que el motor de física utiliza las propiedades de colisión e inercia.  
- Es esencial para tareas de **simulación, visualización y control** en ROS2.