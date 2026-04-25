#!/usr/bin/python3

from pathlib import Path
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chatterbot import ChatBot
import logging

from chatbot.translate_text import translate_to_english
from chatbot.translate_text import translate_to_finnish


logging.basicConfig(level=logging.INFO)


class ChatBotClientNode(Node):
    DEFAULT_RESPONSE = 'I am sorry, but I do not understand.'

    def __init__(self):
        super().__init__('chatbot_node')
        self.publisher_ = self.create_publisher(String, 'chatbot_response', 10)
        self.subscription = self.create_subscription(String, 'recognized_speech', self.tts_callback, 10)

        default_storage_path = Path(tempfile.gettempdir()) / "sop_robot_chatbot.sqlite3"
        storage_path = (
            self.declare_parameter("storage_path", str(default_storage_path))
            .get_parameter_value()
            .string_value
        )
        self.translation_confidence_penalty = (
            self.declare_parameter("translation_confidence_penalty", 0.1)
            .get_parameter_value()
            .double_value
        )

        self.chatbot = ChatBot(
            "Helper",
            read_only=True,
            storage_adapter='chatterbot.storage.SQLStorageAdapter',
            database_uri=f"sqlite:///{storage_path}",
            logic_adapters=[
                {
                    'import_path': 'chatbot.best_match_adapter.BestMatchAdapter',
                    'default_response': self.DEFAULT_RESPONSE,
                    'maximum_similarity_threshold': 0.90,
                    'statement_comparison_function': 'chatterbot.comparisons.LevenshteinDistance',
                },
                {
                    'import_path': 'chatterbot.logic.MathematicalEvaluation',
                }
            ]
        )

    def tts_callback(self, msg):
        try:
            data = self.chatbot_worker_callback(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Chatbot failed to produce a response: {exc}")
            return

        if data != self.DEFAULT_RESPONSE:
            tts_msg = String()
            tts_msg.data = data
            self.publisher_.publish(tts_msg)

    def chatbot_worker_callback(self, recognized_speech):
        original_response = self.chatbot.get_response(recognized_speech)
        original_confidence = original_response.confidence

        translated_input = translate_to_english.translate(recognized_speech)
        translated_response = self.chatbot.get_response(translated_input)
        translated_confidence = translated_response.confidence

        if (translated_confidence - self.translation_confidence_penalty) > original_confidence:
            final_response_text = translate_to_finnish.translate(translated_response.text)
        else:
            final_response_text = original_response.text

        self.get_logger().info(f"Chatbot response: {final_response_text}")
        return final_response_text


def main(args=None):
    rclpy.init(args=args)
    client = ChatBotClientNode()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
