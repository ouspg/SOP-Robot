# LLM Package

This package provides an LLM client for Lehmus AI using OpenAI's SDK.
This package receives speech transcripts from the SST package using the `recognized_speech` topic and after the first turn, corrects likely recognition errors using the conversation history before generating chatbot responses. Chatbot responses are published to `chatbot_response` topic for TTS to read out loud.

  More information about Lehmus AI can be found at [Lehmus AI: Tietoturvallinen Generetiivisen tekoälyn alusta](https://ict.oulu.fi/24009/), [Tervetuloa Lehmus AI-alustalle: Pikakäyttöopas/](https://ict.oulu.fi/24049/).

```text
SST -> LLM correction -> LLM response -> TTS
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

Run the whole chatbot pipeline with the following command:

```terminal
pixi run chatbot
```


#Dependencies

- openai
- python-dotenv
