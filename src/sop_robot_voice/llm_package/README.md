# llm_package

ROS2 LLM package for the SOP Robot voice stack.

Responsibilities:

- load a configured local LLM backend
- keep backend-specific code under `llm_package/backends/`
- manage multi-turn conversation history
- index the local chatbot data into SQLite for direct QA retrieval
- subscribe to `/voice_chatbot/user_text`
- publish `/voice_chatbot/assistant_text`
- expose `/voice_chatbot/clear_history`

Canonical executable:

- `ros2 run llm_package llm_node`

The package is normally started by the canonical robot launch flow via the split
voice-stack include in `chatbot_app/launch/voice_stack.launch.py`.

## Backends

`llm_backend` in `config/voice_chatbot.json` selects the implementation. The
current production backend is:

- `llama-cpp`: local GGUF model through `llama-cpp-python`

The node imports the selected backend through `llm_package.backends`, so adding a
new LLM runtime should not change the ROS node or topic contract.

## Local Knowledge Base

When `llm_knowledge_base_enabled` is true, the node indexes
`legacy/chatbot/chatbot/data` into a local SQLite database before answering.
Retrieved matches are added as context for the LLM prompt. After generation, the
backend compares the LLM response with the strongest SQLite candidate and returns
the closer answer. Exact or high-confidence local matches win only when the LLM
answer is not already close to the local answer.

Important config fields:

- `llm_knowledge_base_data_dir`
- `llm_knowledge_base_path`
- `llm_knowledge_base_recreate`
- `llm_knowledge_base_search_limit`
- `llm_knowledge_base_direct_answer`: allow the answer selector to return the
  local SQLite answer when it is closer than the generated answer

Useful checks:

```bash
pixi run test-llm-knowledge
pixi run test-llm-knowledge-learning
pixi run test-llm-knowledge-correct
pixi run generate-llm-knowledge-eval
pixi run test-llm-knowledge-compare
pixi run test-llm-models
pixi run try-llm-model -- --model ahma=llama-cpp:/home/aapot/SOP-Robot/models/Ahma-2-4B-Instruct.Q4_K_S.gguf --prompt "Kerro yhdellä lauseella Oulusta."
pixi run benchmark-llm-knowledge
pixi run benchmark-llm-models
pixi run benchmark-llm-text-models
```

`benchmark-llm-text-models` benchmarks the local Ahma and Llama 3.1 GGUF files
plus `unsloth/gemma-4-E4B-it-GGUF` through `llama-cpp-python` text-only chat
calls.

## CUDA Offload

On WSL2/NVIDIA systems, run `pixi run install-llama-cuda` after `pixi install`.
This rebuilds `llama-cpp-python` with CUDA support and verifies that llama.cpp
reports GPU offload support. The editable voice config uses
`llm_n_gpu_layers: -1` to request full GGUF layer offload.
