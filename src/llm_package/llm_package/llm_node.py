import os

import rclpy
from llama_cpp import Llama
from rclpy.node import Node
from std_msgs.msg import Bool, String

MODEL_REPOSITORY = 'mradermacher/Ahma-2-4B-Instruct-GGUF'
MODEL_FILE = 'Ahma-2-4B-Instruct.Q4_K_M.gguf'
MAX_HISTORY_MESSAGES = 8

SYSTEM_PROMPT = (
    'Olet ystävällinen suomea puhuva humanoidirobotti. Vastaa luontevasti ja '
    'tiiviisti, yleensä enintään kolmella virkkeellä. Vastauksesi luetaan '
    'ääneen, joten älä käytä markdownia, luetteloita, emojeita tai '
    'erikoismerkintöjä.'
)


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self.response_publisher = self.create_publisher(
            String,
            'chatbot_response',
            10,
        )
        self.can_listen_publisher = self.create_publisher(
            Bool,
            'can_listen',
            10,
        )
        self.speech_subscription = self.create_subscription(
            String,
            'recognized_speech',
            self.speech_callback,
            10,
        )

        self.get_logger().info(
            f'Loading {MODEL_REPOSITORY}/{MODEL_FILE} on the GPU...'
        )
        self.model = Llama.from_pretrained(
            repo_id=MODEL_REPOSITORY,
            filename=MODEL_FILE,
            n_gpu_layers=-1,
            n_ctx=4096,
            n_batch=512,
            n_threads=max(2, min(8, (os.cpu_count() or 4) // 2)),
            flash_attn=True,
            verbose=False,
        )
        self.history = [
            {
                'role': 'system',
                'content': SYSTEM_PROMPT,
            }
        ]
        self.get_logger().info('LLM ready.')

    def speech_callback(self, message):
        prompt = message.data.strip()
        if not prompt:
            return

        self.can_listen_publisher.publish(Bool(data=False))
        try:
            answer = self.generate_response(prompt)
        except Exception as error:
            self.get_logger().error(f'LLM generation failed: {error}')
            self.can_listen_publisher.publish(Bool(data=True))
            return

        self.response_publisher.publish(String(data=answer))

    def generate_response(self, prompt):
        self.get_logger().info(f'User: {prompt}')
        self.history.append({
            'role': 'user',
            'content': prompt,
        })

        response = self.model.create_chat_completion(
            messages=self.history,
            temperature=0.65,
            max_tokens=160,
            repeat_penalty=1.1,
        )
        answer = str(
            response['choices'][0]['message']['content'] or ''
        ).strip()
        if not answer:
            raise RuntimeError('The model returned an empty response.')

        self.history.append({
            'role': 'assistant',
            'content': answer,
        })
        self.history = [self.history[0]] + self.history[1:][
            -MAX_HISTORY_MESSAGES:
        ]
        self.get_logger().info(f'Robot: {answer}')
        return answer


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
