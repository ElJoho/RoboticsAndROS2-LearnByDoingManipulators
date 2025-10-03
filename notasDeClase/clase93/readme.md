# Clase 93 — Interfaz Alexa ↔ Flask ↔ ROS 2 (arduinobot_remote)

> **Objetivo de la clase:** crear un micro‑servicio web en **Python (Flask)** que conecte una **Skill de Amazon Alexa (ASK SDK)** con nuestro ecosistema **ROS 2**, iniciando con un *LaunchRequest* que responde “Hi, how can I help?” y dejando listo el canal para futuros *intents* que enviarán mensajes al *task server*.

---

## 1) Script principal: `alexa_interface.py` (qué hace y cómo está organizado)

**Rol del script.** Expone un servidor HTTP con **Flask** en `localhost:5000`, registra un **SkillAdapter** del **ASK SDK** y atiende solicitudes de Alexa. Implementa el primer *handler* (`LaunchRequestHandler`) que responde cuando el usuario inicia la interacción (p. ej. “Alexa, activa el robot”).

**Componentes clave:**
- `Flask(app)`: servidor web local donde Alexa (vía Ngrok) envía las peticiones.
- `SkillBuilder (sb)`: contenedor donde se registran los *request handlers* (intents, launch, etc.).
- `SkillAdapter`: puente entre Flask y el *Skill* creado con el `SkillBuilder`; necesita el `skill_id` de la consola de Alexa.
- `LaunchRequestHandler`: responde al evento de inicio (**LaunchRequest**) diciendo “Hi, how can I help”.

**Rutas y *binding* web:**
- `@app.route("/")` e igualmente `skill_adapter.register(app=app, route="/")`: configuran que las peticiones HTTP a `/` se enruten al adaptador de Alexa.
- `if __name__ == "__main__": app.run()`: levanta el servidor Flask (por defecto en `127.0.0.1:5000`).

### 1.1 Pseudocódigo del script

```text
INICIO
  importar Flask y librerías ASK SDK (SkillBuilder, SkillAdapter, helpers)
  crear instancia Flask: app = Flask(__name__)

  DEFINIR clase LaunchRequestHandler (hereda de AbstractRequestHandler):
    can_handle(handler_input):
      retornar si el tipo de request es "LaunchRequest"
    handle(handler_input):
      speech_text ← "Hi, how can i help"
      construir respuesta de voz y tarjeta (SimpleCard)
      set_should_end_session(False)   # mantiene la sesión abierta
      retornar respuesta

  sb ← SkillBuilder()
  registrar LaunchRequestHandler en sb

  skill ← sb.create()
  skill_adapter ← SkillAdapter(skill=skill, skill_id="SKILL_ID", app=app)

  DEFINIR ruta HTTP "/":
    retornar skill_adapter.dispatch_request()  # procesa JSON de Alexa

  registrar explícitamente el adaptador en "/" (skill_adapter.register)

  SI se ejecuta como script principal:
    app.run()  # iniciar servidor en 127.0.0.1:5000
FIN
```

> **Notas de implementación**
> - `SkillAdapter` necesita el **Skill ID** exacto copiado de la consola de Alexa.
> - `set_should_end_session(False)` mantiene la conversación abierta para recibir siguientes **intents**.
> - Los futuros *handlers* (intents) deberán registrarse con `sb.add_request_handler(...)` antes de crear el `skill_adapter`.

---

## 2) Flujo de trabajo en tiempo de ejecución (peticiones)

1. Usuario dice: “**Alexa, activa el robot**”.  
2. Alexa envía una **LaunchRequest** a la URL pública (expuesta por **Ngrok**) → Ngrok la reenvía a `http://localhost:5000/`.  
3. Flask recibe `POST /` → `SkillAdapter` decodifica el JSON y despacha al `LaunchRequestHandler`.  
4. `LaunchRequestHandler` construye la respuesta “**Hi, how can I help**” y la devuelve a Alexa.  
5. Alexa pronuncia la respuesta y la sesión continúa abierta (pueden venir nuevos *intents*).

---

## 3) Terminales y comandos (qué se hizo y para qué)

### **Terminal 1 — Compilación ROS 2 y ejecución del servidor Flask**

```bash
cd arduinobot_ws/
colcon build
cd src/
cd arduinobot_remote/arduinobot_remote/
ls
chmod +x alexa_interface.py
ls
python3 alexa_interface.py
```
**Explicación de cada comando:**
- `cd arduinobot_ws/`: entra al *workspace* de ROS 2.
- `colcon build`: compila los paquetes del *workspace* (verificación de que `arduinobot_remote` y dependencias construyen sin errores).
- `cd src/` → `cd arduinobot_remote/arduinobot_remote/`: navega hasta la carpeta del paquete y su módulo Python.
- `ls`: lista archivos para confirmar presencia de `alexa_interface.py` y permisos.
- `chmod +x alexa_interface.py`: marca el script como ejecutable (útil si se desea invocarlo como `./alexa_interface.py`; con `python3` no es estrictamente necesario, pero es buena práctica).
- `python3 alexa_interface.py`: **inicia el servidor Flask**. La consola muestra:
  - “*Serving Flask app 'alexa_interface'*”,  
  - “*Running on http://127.0.0.1:5000*”,  
  - y cada petición entrante como líneas de log `POST /` con código 200.

### **Terminal 2 — Exponer el servidor local con Ngrok**

La pantalla de Ngrok muestra, por ejemplo:

```
Forwarding  https://overexpectantly-noneclipsing-melany.ngrok-free.dev -> http://localhost:5000
```

**Propósito:**
- Crear un **túnel seguro** desde Internet hacia `http://localhost:5000`, permitiendo que la consola de Alexa envíe solicitudes a tu servidor Flask.
- El comando típico es:
  ```bash
  ngrok http 5000
  ```
  (Dependiendo de la instalación, puede ejecutarse como `./ngrok http 5000` desde la carpeta donde está el binario).

**Acciones posteriores imprescindibles:**
- Copiar la **URL pública HTTPS** que muestra Ngrok y **pegarla en el Endpoint** de la Skill en la consola de **Alexa Developer** (sección *Endpoint → Default Region*).  
- Guardar los cambios en la consola y realizar el *Test* de la Skill.  
- Recuerda: en el plan gratuito, **la URL cambia** cada vez que reinicias Ngrok; si cambias la URL, **actualiza** el Endpoint en la consola.

---

## 4) Requisitos y dependencias

- Python 3, `flask`, `ask-sdk-core`, `flask-ask-sdk`.  
- Ngrok instalado y autenticado (opcional pero necesario para pruebas desde la consola de Alexa).  
- `skill_id` válido copiado de la consola de Alexa y colocado en el script.

> **ROS 2** aún no se invoca desde este script. En clases siguientes se añadirán *intents* que publiquen o llamen acciones/servicios hacia el *task server*.

---

## 5) Verificación rápida

- Con `python3 alexa_interface.py` en ejecución, abre el *dashboard* de Ngrok (`http://127.0.0.1:4040`) y lanza una prueba desde la consola de Alexa: deberías ver un `POST /` en Terminal 1 con `200` y oír en Alexa: **“Hi, how can I help?”**.

---

## 6) Problemas frecuentes y tips

- **403/Unauthorized desde Alexa**: revisa que el **Skill ID** en el script coincida exactamente con el de la consola.  
- **404 en Flask**: confirma que la ruta sea `/` y que `skill_adapter.register(app=app, route="/")` esté presente.  
- **Alexa no llega a tu PC**: verifica que Ngrok esté apuntando a `5000` y que el Endpoint de la Skill use la **URL HTTPS** actual.  
- **Colcon build no ve cambios Python**: si agregas módulos nuevos al paquete, confirma que estén incluidos en `setup.py`/`package.xml` (cuando corresponda).

---

## 7) Próximos pasos

- Añadir *intent handlers* (ej. `MoveToHomeIntentHandler`, `OpenGripperIntentHandler`, etc.).  
- Enlazar cada *intent* con ROS 2 (publicadores/servicios/acciones) para que el robot ejecute tareas mediante voz.  
- Escribir pruebas unitarias básicas para los *handlers* y validaciones de payload de Alexa.

---

### Anexo — Estructura mínima sugerida del paquete

```text
arduinobot_remote/
└── arduinobot_remote/
    ├── __init__.py
    ├── alexa_interface.py   # servidor Flask + SkillAdapter + handlers
    └── task_server.py       # (desde clases previas) acciones hacia el robot
```