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

Copy the `.env.example` file to `.env.local` and fill in your Lehmus AI credentials.

```terminal
cp .env.example .env.local
```

The default model is `openai/gpt-oss-120b` it's Lehmus AI model id is `azsydsttjnlbfjbgqnwd`
Change the API key in `.env.local` to your Lehmus AI key.

```dotenv
LLM_BASE_URL=https://api.lehmus-ai.oulu.fi/v1
LLM_API_KEY=your-lehmusai-key
LLM_MODEL=azsydsttjnlbfjbgqnwd
```

Run the LLM client with the following command:

```terminal
pixi run llm
```

Start the namespaced speech nodes with the following command (the full demo supplies
the coordinator that routes messages between them):

```terminal
pixi run chatbot
```


## Dependencies

- openai
- python-dotenv

```


#Dependencies

- openai
- python-dotenv
