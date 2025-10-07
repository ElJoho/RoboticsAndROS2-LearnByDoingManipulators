#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler

# --- ROS 2 imports ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# FIX 1: correct action import path (ROS 2 actions live under <pkg>.action)
from arduinobot_msgs.action import ArduinobotTask  # was: from arduinobot_msgs import ArduinobotTask

import threading

# --- ROS 2 init & node spin in background ---
# FIX 2: create a persistent node and spin it so ActionClient can work.
rclpy.init(args=None)
_alexa_node = Node("alexa_interface")  # keep a strong reference so it isn't GC'd

def _spin_node():
    # spinning in a daemon thread lets Flask run in the main thread
    try:
        rclpy.spin(_alexa_node)
    except Exception as _e:
        # optional: print for visibility if something goes wrong spinning
        print(_e)

threading.Thread(target=_spin_node, daemon=True).start()

# Action client bound to the persistent node
action_client = ActionClient(_alexa_node, ArduinobotTask, "task_server")

# --- Flask / Alexa ---
app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, how can i help"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)

        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        # send goal asynchronously; completion handled by the action server
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I am moving."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)
        ).set_should_end_session(True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response


class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, see you later."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)
        ).set_should_end_session(True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 2
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    

class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, I am ready."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)
        ).set_should_end_session(True)
        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I didn't get it. Can you please say it again!?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

    
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.666ccb70-55dd-4916-93cf-0e63beb79b4d", app=app)

# Keep only the SkillAdapter route (avoids double-binding "/" in Flask).
# If you also had a custom @app.route("/") view, remove it.
skill_adapter.register(app=app, route="/")


if __name__ == "__main__":
    app.run()
