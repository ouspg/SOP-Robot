This package contains service and client for finnish text-to-speech feature. Service will automatically play synthesized speech when called with wanted sentence as an argument.

# Usage

## Before
Check that `model.pth` and `config.json` are located in
`src/tts_package/resource/`. Pixi downloads these with:

```console
pixi run download-tts-model
```

## Dependencies

* `coqui-tts`

Pixi installs the Python dependencies.

## Run TTS service

```console
pixi run tts
```


## Run TTS client

```console
pixi run tts-client
```

```console
Input command: Hei. Tässä on lause joka syntentisoidaan puheeksi.
```
Service will now try to synthentize sentence into .wav file located in 'src/tts_package/resource/output.wav' which will then be played automatically.

## Potential future improvements

* Implement this feature to work with potential speech-to-text and chatbot features.
