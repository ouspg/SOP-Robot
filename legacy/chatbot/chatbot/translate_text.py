#!/usr/bin/python3

from translate import Translator

class translate_to_english:
    @staticmethod
    def translate(text, source_language='fi', target_language='en'):
        translated_text = Translator(
            to_lang=target_language,
            from_lang=source_language,
        ).translate(text)
        return translated_text

class translate_to_finnish:
    @staticmethod
    def translate(text, source_language='en', target_language='fi'):
        translated_text = Translator(
            to_lang=target_language,
            from_lang=source_language,
        ).translate(text)
        return translated_text

