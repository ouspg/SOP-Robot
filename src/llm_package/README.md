# LLM Package

This package provides an LLM client for Lehmus AI using OpenAI's SDK.
This package receives speech transcripts on `recognized_speech` and publishes responses on `chatbot_response`.
The robot chatbot launch places the speech nodes in namespaces matching their package names:
`/sst_package`, `/llm_package`, and `/tts_package` (equivalent to passing
`--ros-args -r __ns:=/llm_package` to the LLM node).
In the full demo, `full_demo` forwards `/sst_package/recognized_speech` to
`/llm_package/recognized_speech` and `/llm_package/chatbot_response` to
`/tts_package/chatbot_response`. It controls listening through `/sst_package/can_listen`
and monitors TTS readiness on `/tts_package/can_listen`.

  More information about Lehmus AI can be found at [Lehmus AI: Tietoturvallinen Generetiivisen tekoälyn alusta](https://ict.oulu.fi/24009/), [Tervetuloa Lehmus AI-alustalle: Pikakäyttöopas/](https://ict.oulu.fi/24049/).

```text
SST -> LLM response -> TTS
```
## Usage

Copy the `.env.example` file to `.env.local` and set your Lehmus AI API key. The key is
the only thing kept in `.env.local`; it is also used by the SST node.

```terminal
cp .env.example .env.local
```

```dotenv
LLM_API_KEY=your-lehmusai-key
```

All other settings are in [`config/llm.yaml`](../../config/llm.yaml): base URL, model
(default `openai/gpt-oss-120b`, Lehmus AI id `azsydsttjnlbfjbgqnwd`), token budget,
conversation memory length and the RAG settings (`rag:`). The prompts are in
`llm_package/llm_node.py`.

### Knowledge (RAG)

For short Q&A data in `RAG/Questions`, it is included in the system prompt.

The documents in `RAG/*.txt` are the RAG's knowledge base which is uploaded to Lehmus AI RAG.
Answers are retrieved from the RAG endpoint for every question. 

## Uploading RAG Knowledge
Files in `RAG/` are not automatically kept in sync with Lehmus AI RAG.
After making changes or adding files to `RAG/`. Upload them to Lehmus AI RAG from Lehmus AI portal.

Run the LLM client with the following command:

```terminal
pixi run llm
```

Test with text input from the terminal

```terminal
pixi run llm-client
```

Run the voice chatbot on its own (SST, LLM and TTS connected directly, without face
tracking) with the following command:

```terminal
pixi run chatbot
```

`pixi run chatbot-nodes` starts the same namespaced nodes without connecting them, so the
full demo's `full_demo` coordinator can route messages between them. The `robot-demo-real`
and `robot-demo-fake` scripts use it.


## Dependencies

- openai
- python-dotenv

```


#Dependencies

- openai
- python-dotenv
