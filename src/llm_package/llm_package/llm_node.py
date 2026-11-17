import os
import time
from pathlib import Path

import httpx
import rclpy
import yaml
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import Bool, String

DEFAULT_API_KEY = 'your-lehmusai-key'
CONFIG_PATH = Path('config') / 'llm.yaml'


RESPONSE_PROMPT = (
    'Olet ystävällinen suomea puhuva humanoidirobotti nimeltä Robotti Roope. '
    'Vastaa luontevasti ja tiiviisti, yleensä enintään 1 tai 2 virkkeellä. '
    'Vastauksesi luetaan ääneen, joten älä käytä markdownia, lyhenteitä, luetteloita, '
    'emojeita tai erikoismerkintöjä. '
    'Sano jokainen luku vain kerran. Kirjoita yksiköt ja merkit sanoina, '
    'esimerkiksi euroa kuukaudessa ja prosenttia. '
    'Älä mainitse sanaa konteksti. Älä mainitse metatietoja tai lähteitä. '
    'Älä kerro mihin perustat vastauksesi. Vastaa aina niin kuin tieto olisi sinun omaasi. '
    'Älä ikinä mainitse lähteitä tai mistä olet saanut tiedon.'
)
KNOWLEDGE_PROMPT = (
    'Alla on taustatietoa, jota käytät vastauksissasi. '
    'Jos kysymys koskee yliopistoa, opintoja tai hakemista eikä vastausta löydy '
    'taustatiedosta, älä keksi yksityiskohtia.'
)
QA_HEADER = 'Usein kysytyt kysymykset ja niiden vastaukset:'
RETRIEVED_HEADER = 'Kysymykseen liittyvää taustatietoa:'

def load_local_environment():
    project_root = Path(os.environ.get('SOP_ROBOT_ROOT', Path.cwd())).resolve()
    load_dotenv(project_root / '.env.local')
    return project_root


def load_config(project_root):
    with open(project_root / CONFIG_PATH, encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        project_root = load_local_environment()
        config = load_config(project_root)
        llm_config = config['llm']
        self.base_url = llm_config['base_url']
        self.model = llm_config['model']
        self.max_tokens = int(llm_config['max_tokens'])
        self.max_history_messages = int(llm_config['max_history_messages'])
        api_key = os.environ.get('LLM_API_KEY', DEFAULT_API_KEY)

        # RAG: the curated Q&A goes into the system prompt, documents are retrieved per
        # question from the Lehmus AI RAG endpoint.
        rag_config = config['rag']
        self.rag_url = f'{self.base_url.rstrip("/")}/api/{rag_config["id"]}/context'
        self.rag_max_chunks = int(rag_config['max_chunks'])
        self.rag_client = httpx.Client(
            headers={'Authorization': f'Bearer {api_key}'},
            timeout=float(rag_config['timeout_seconds']),
        )
        self.system_prompt = self.build_system_prompt(project_root / rag_config['questions_file'])

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
            f'LLM ready: model={self.model}, retrieving context from {self.rag_url}.'
        )

    def speech_callback(self, message):
        transcript = message.data.strip()
        if not transcript:
            return

        self.can_listen_publisher.publish(Bool(data=False))
        try:
            self.get_logger().info(f'Received from SST: {transcript}')
            answer = self.answer_user(transcript)
            self.get_logger().info(f'Robot: {answer}')
        except Exception as error:
            self.get_logger().error(f'LLM generation failed: {error}')
            self.can_listen_publisher.publish(Bool(data=True))
            return

        self.response_publisher.publish(String(data=answer))

    def build_system_prompt(self, questions_file):
        """Return the response prompt with the curated Q&A appended."""
        if not questions_file.is_file():
            self.get_logger().warning(f'{questions_file} not found; no Q&A in the prompt.')
            return RESPONSE_PROMPT
        questions = questions_file.read_text(encoding='utf-8').strip()
        return f'{RESPONSE_PROMPT}\n\n{KNOWLEDGE_PROMPT}\n\n{QA_HEADER}\n{questions}'

    def retrieve_context(self, transcript):
        """Return retrieved passages as a system message, or None if nothing was found."""
        # Include the previous user turn for follow-up questions.
        previous_questions = [m['content'] for m in self.history if m['role'] == 'user']
        query = ' '.join([*previous_questions[-1:], transcript])
        try:
            response = self.rag_client.post(
                self.rag_url,
                json={'query': query, 'max_chunks': self.rag_max_chunks},
            )
            response.raise_for_status()
            chunks = [str(chunk).strip() for chunk in response.json().get('chunks', [])]
        except (httpx.HTTPError, ValueError) as error:
            self.get_logger().warning(f'RAG retrieval failed, answering without it: {error}')
            return None

        context = '\n\n'.join(chunk for chunk in chunks if chunk)
        if not context:
            return None
        return {'role': 'system', 'content': f'{RETRIEVED_HEADER}\n{context}'}

    def answer_user(self, transcript):
        messages = [
            {'role': 'system', 'content': self.system_prompt},
            *self.history,
        ]
        started = time.monotonic()
        context_message = self.retrieve_context(transcript)
        retrieved = time.monotonic()
        if context_message:
            messages.append(context_message)
        messages.append({'role': 'user', 'content': transcript})

        answer = self.complete_answer(
            messages,
            max_tokens=self.max_tokens,
            empty_error='The model response was empty.',
            )

        self.get_logger().info(
            f'Answered in {time.monotonic() - started:.2f} s '
            f'(retrieval {retrieved - started:.2f} s).'
        )

        self.history.extend(
            [
                {
                    'role': 'user',
                    'content': transcript,
                },
                {
                    'role': 'assistant',
                    'content': answer,
                },
            ]
        )
        self.history = self.history[-self.max_history_messages :]
        return answer


    def complete_answer(self, messages, max_tokens, empty_error):
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

            raise RuntimeError(f'{empty_error} finish_reason={finish_reason!r}.')

    def destroy_node(self):
        self.client.close()
        self.rag_client.close()
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
