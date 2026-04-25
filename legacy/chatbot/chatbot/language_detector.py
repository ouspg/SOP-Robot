#!/usr/bin/python3

from langdetect import detect

class detect_language():

    @staticmethod
    def detect(text):
        try:
            language = detect(text)
        except Exception:
            # Handle exception if language detection fails
            language = 'unknown'

        return language
