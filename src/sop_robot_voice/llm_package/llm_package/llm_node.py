"""ROS2 LLM node for the SOP Robot voice stack."""

from __future__ import annotations

import queue
import threading
import traceback

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_stack_common.contracts import (
    ASSISTANT_TEXT_TOPIC,
    CLEAR_HISTORY_SERVICE,
    STATUS_ERROR,
    STATUS_INITIALIZING,
    STATUS_LLM_RESPONDING,
    STATUS_READY,
    USER_TEXT_TOPIC,
)
from voice_stack_common.ros_helpers import VoiceNodeBase

from .llm_engine import ChatLLM


def _describe_llm_gpu_offload(config) -> tuple[bool, str]:
    requested_layers = int(getattr(config, "llm_n_gpu_layers", 0))
    if requested_layers == 0:
        return False, "LLM GPU offload is disabled by config (`llm_n_gpu_layers = 0`)."

    try:
        import llama_cpp.llama_cpp as llama_cpp_backend
    except Exception as exc:
        return False, f"Could not import llama-cpp-python: {exc}"

    if not llama_cpp_backend.llama_supports_gpu_offload():
        return False, "llama-cpp-python was built without CUDA GPU offload support."

    if requested_layers < 0:
        return True, "LLM GPU offload enabled: all supported layers will run on CUDA."

    return True, f"LLM GPU offload enabled: {requested_layers} layers requested on CUDA."


class LlmNode(VoiceNodeBase):
    """Multi-turn chat inference node."""

    def __init__(self) -> None:
        super().__init__("llm_package")
        self._init_base()

        self._user_sub = self.create_subscription(
            String, USER_TEXT_TOPIC, self._on_user_text, 10
        )
        self._assistant_pub = self.create_publisher(String, ASSISTANT_TEXT_TOPIC, 10)
        self._clear_srv = self.create_service(
            Trigger, CLEAR_HISTORY_SERVICE, self._on_clear_history
        )

        self._request_queue: queue.Queue[str] = queue.Queue()
        self._running = threading.Event()
        self._running.set()
        self._request_thread: threading.Thread | None = None
        self._llm: ChatLLM | None = None

        self._initialize()

    def _initialize(self) -> None:
        try:
            self._publish_status(STATUS_INITIALIZING)
            llm_gpu_ok, llm_gpu_message = _describe_llm_gpu_offload(self._config)
            if llm_gpu_ok:
                self._publish_log(llm_gpu_message)
            else:
                self.get_logger().warning(llm_gpu_message)
                self._publish_log(llm_gpu_message)
            self._publish_log("LLM node: initializing LLM...")
            self._llm = ChatLLM(self._config)

            self._request_thread = threading.Thread(
                target=self._request_loop,
                name="chat-request-loop",
                daemon=True,
            )
            self._request_thread.start()

            self._publish_log("LLM node ready.")
            self._publish_status(STATUS_READY)
        except Exception as exc:
            self._publish_status(STATUS_ERROR)
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
                self._publish_status(STATUS_LLM_RESPONDING)
                response = self._llm.chat(user_text)
                if response:
                    self._assistant_pub.publish(String(data=response))
                    self._publish_log(f"Assistant: {response}")
                self._publish_status(STATUS_READY)
            except Exception:
                self._publish_status(STATUS_ERROR)
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
        node = LlmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
