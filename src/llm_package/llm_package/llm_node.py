# Copyright 2026 Aapo2001
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

import rclpy
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import Bool, String

DEFAULT_BASE_URL = 'https://openrouter.ai/api/v1'
DEFAULT_MODEL = 'openai/gpt-oss-120b'
DEFAULT_API_KEY = 'not-needed'
MAX_HISTORY_MESSAGES = 8

CORRECTION_PROMPT = (
    'Korjaa viimeisin Faster Whisperin tuottama suomenkielinen litterointi. '
    'Päättele keskusteluhistorian avulla, mitä käyttäjä todennäköisimmin '
    'sanoi. Säilytä käyttäjän tarkoitus, älä lisää uutta sisältöä äläkä '
    'vastaa käyttäjälle. Palauta vain korjattu käyttäjän lause ilman '
    'selityksiä, lainausmerkkejä tai markdownia.'
)

RESPONSE_PROMPT = (
    'Olet ystävällinen suomea puhuva humanoidirobotti nimeltä Robotti Roope. '
    'Vastaa luontevasti ja '
    'tiiviisti, yleensä enintään kolmella virkkeellä. Vastauksesi luetaan '
    'ääneen, joten älä käytä markdownia, luetteloita, emojeita tai '
    'erikoismerkintöjä.'
)


def load_local_environment():
    project_root = Path(
        os.environ.get('SOP_ROBOT_ROOT', Path.cwd())
    ).resolve()
    load_dotenv(project_root / '.env.local')


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        load_local_environment()
        self.base_url = os.environ.get(
            'LLM_BASE_URL',
            DEFAULT_BASE_URL,
        ) or DEFAULT_BASE_URL
        self.model = os.environ.get('LLM_MODEL') or DEFAULT_MODEL
        api_key = os.environ.get('LLM_API_KEY') or DEFAULT_API_KEY

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

        self.client = OpenAI(
            base_url=self.base_url,
            api_key=api_key,
        )
        self.history = []
        self.get_logger().info(
            f'OpenAI-compatible client ready with {self.model}.'
        )

    def speech_callback(self, message):
        transcript = message.data.strip()
        if not transcript:
            return

        self.can_listen_publisher.publish(Bool(data=False))
        try:
            answer = self.generate_response(transcript)
        except Exception as error:
            self.get_logger().error(f'LLM generation failed: {error}')
            self.can_listen_publisher.publish(Bool(data=True))
            return

        self.response_publisher.publish(String(data=answer))

    def generate_response(self, transcript):
        self.get_logger().info(
            f'Faster Whisper: {transcript}'
        )
        if self.history:
            corrected = self.correct_transcript(transcript)
            self.get_logger().info(f'Interpreted user: {corrected}')
        else:
            corrected = transcript
            self.get_logger().info(
                'First turn: using the Faster Whisper transcript directly.'
            )
        answer = self.answer_user(corrected)
        self.get_logger().info(f'Robot: {answer}')
        return answer

    def correct_transcript(self, transcript):
        messages = [{
            'role': 'system',
            'content': CORRECTION_PROMPT,
        }] + self.history + [{
            'role': 'user',
            'content': (
                'Faster Whisperin litterointi:\n'
                f'{transcript}'
            ),
        }]
        return self.complete(
            messages,
            max_tokens=512,
            empty_error='The transcript correction was empty.',
        )

    def answer_user(self, corrected):
        messages = [{
            'role': 'system',
            'content': RESPONSE_PROMPT,
        }] + self.history + [{
            'role': 'user',
            'content': corrected,
        }]
        answer = self.complete(
            messages,
            max_tokens=512,
            empty_error='The model response was empty.',
        )

        self.history.extend([{
            'role': 'user',
            'content': corrected,
        }, {
            'role': 'assistant',
            'content': answer,
        }])
        self.history = self.history[-MAX_HISTORY_MESSAGES:]
        return answer

    def complete(self, messages, max_tokens, empty_error):
        finish_reason = None
        for token_budget in (max_tokens, max_tokens * 2):
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=token_budget,
            )
            choice = response.choices[0]
            text = str(choice.message.content or '').strip()
            if text:
                return text

            finish_reason = choice.finish_reason
            if finish_reason != 'length':
                break
            self.get_logger().warning(
                'The LLM exhausted its output budget before returning '
                f'text; retrying with {token_budget * 2} tokens.'
            )

        raise RuntimeError(
            f'{empty_error} finish_reason={finish_reason!r}.'
        )

    def destroy_node(self):
        self.client.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
