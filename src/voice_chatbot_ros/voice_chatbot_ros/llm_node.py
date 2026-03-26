"""
ROS 2 LLM node – multi-turn chat inference.

Subscribes to user text, publishes assistant replies.  A request
queue serialises access to the ChatLLM instance.
"""

import queue
import threading
import traceback

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_chatbot.llm import ChatLLM
from voice_chatbot_ros._base import VoiceNodeBase


class VoiceLlmNode(VoiceNodeBase):
    def __init__(self) -> None:
        super().__init__("voice_llm")
        self._init_base()

        self._user_sub = self.create_subscription(
            String, "user_text", self._on_user_text, 10
        )
        self._assistant_pub = self.create_publisher(String, "assistant_text", 10)
        self._clear_srv = self.create_service(
            Trigger, "clear_history", self._on_clear_history
        )

        self._request_queue: queue.Queue[str] = queue.Queue()
        self._running = threading.Event()
        self._running.set()
        self._request_thread: threading.Thread | None = None
        self._llm: ChatLLM | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status("initializing")
            self._publish_log("LLM node: initializing LLM...")
            self._llm = ChatLLM(self._config)

            self._request_thread = threading.Thread(
                target=self._request_loop, name="chat-request-loop", daemon=True
            )
            self._request_thread.start()

            self._publish_log("LLM node ready.")
            self._publish_status("ready")
        except Exception as exc:
            self._publish_status("error")
            self._publish_log(f"LLM node initialization failed: {exc}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _on_user_text(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self._publish_log(f"Queued user text: {text}")
            self._request_queue.put(text)

    def _on_clear_history(self, request, response):
        del request
        if self._llm is None:
            response.success = False
            response.message = "LLM is not initialized."
            return response
        self._llm.clear_history()
        self._publish_log("Conversation history cleared.")
        response.success = True
        response.message = "Conversation history cleared."
        return response

    def _request_loop(self) -> None:
        while self._running.is_set():
            try:
                user_text = self._request_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self._llm is None:
                    continue
                self._publish_status("llm_responding")
                response = self._llm.chat(user_text)
                self._assistant_pub.publish(String(data=response))
                self._publish_log(f"Assistant: {response}")
                self._publish_status("ready")
            except Exception:
                self._publish_status("error")
                self._publish_log("LLM request processing failed.")
                self.get_logger().error(traceback.format_exc())

    def destroy_node(self) -> None:
        self._running.clear()
        if self._request_thread is not None and self._request_thread.is_alive():
            self._request_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceLlmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
