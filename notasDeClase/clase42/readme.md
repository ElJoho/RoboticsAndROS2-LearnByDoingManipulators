# Clase 42 – Simulación del Robot en Gazebo

## 🌍 ¿Qué es un motor de física?

Un **motor de física** es un software capaz de simular fenómenos físicos como:
- Gravedad
- Fricción
- Aceleración
- Colisiones entre objetos  

De esta forma, se puede reproducir el comportamiento de objetos sometidos a fuerzas, incluyendo robots y sus componentes.  

En robótica, **Gazebo** es uno de los simuladores más utilizados y fáciles de integrar con **ROS 2**. Permite:
- Simular el movimiento del robot bajo leyes físicas reales.  
- Simular sensores (cámaras, LIDAR, etc.).  
- Crear entornos virtuales (casas, oficinas, escuelas) donde interactuar con el robot.  

---

## 🤖 Integración con ROS 2

En un robot real:
- **Actuadores (motores)** publican mensajes en un *topic* describiendo su estado (posición, ángulo, velocidad).  
- **Sensores (ej. cámaras)** publican datos en *topics* para que otros nodos puedan utilizarlos.  
- **Actuadores** se suscriben a *topics* para recibir comandos de movimiento.  
- Herramientas como **RViz** se suscriben a estos *topics* para mostrar la información en pantalla.

En simulación con **Gazebo**:
- Gazebo **reemplaza al hardware real** y publica los mismos mensajes en los mismos *topics*.  
- Los demás nodos de ROS 2 **no distinguen** si la información viene de un robot real o de uno simulado.  
- Así, podemos ejecutar los mismos nodos y programas sin necesidad de tener el robot físico.  

---

## 🛠️ Ventajas de usar Gazebo

1. **Probar nodos sin hardware real**  
   Se pueden desarrollar y testear algoritmos antes de tener el robot físico.  

2. **Reutilización de código**  
   Una vez construido el robot real, el mismo código probado en Gazebo puede ejecutarse directamente en el hardware sin cambios, ya que la interfaz es la misma.  

3. **Visualización y depuración**  
   Con RViz podemos mostrar datos de sensores simulados exactamente igual que si fueran reales.  