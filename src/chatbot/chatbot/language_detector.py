#!/usr/bin/python3

from langdetect import detect

class detect_language():
    
    def detect(text):
        try:
            language = detect(text)
        except Exception as e:
            # Handle exception if language detection fails
            language = 'unknown'

        return language

