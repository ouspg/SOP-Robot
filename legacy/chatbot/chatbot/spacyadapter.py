#!/usr/bin/python3

import spacy
from chatterbot import filters
from chatterbot.conversation import Statement
from chatterbot.logic import LogicAdapter
import numpy as np


class SpacyBestMatch(LogicAdapter):
    """
    A logic adapter that returns a response based on known responses to
    the closest matches to the input statement using spaCy word vectors.
    """

    def __init__(self, chatbot, **kwargs):
        super().__init__(chatbot, **kwargs)

        self.nlp = spacy.load("en_core_web_md")
        self.excluded_words = kwargs.get('excluded_words')
        self.maximum_similarity_threshold = kwargs.get('maximum_similarity_threshold', 0.8)

    def process(self, statement, additional_response_selection_parameters=None):
        input_vec = np.asarray(self.nlp(statement.text).vector, dtype=np.float32)
        input_norm = float(np.linalg.norm(input_vec))
        if input_norm == 0.0:
            return self.get_default_response(statement)

        closest_match = None
        highest_confidence = 0.0
        
        for candidate in self.chatbot.storage.filter(in_response_to=None):
            statement_vec = np.asarray(self.nlp(candidate.text).vector, dtype=np.float32)
            statement_norm = float(np.linalg.norm(statement_vec))
            if statement_norm == 0.0:
                continue

            confidence = float(np.dot(input_vec, statement_vec) / (input_norm * statement_norm))

            if confidence > highest_confidence:
                highest_confidence = confidence
                closest_match = candidate

                if confidence >= 0.95:
                    break

        if closest_match is not None:
            self.chatbot.logger.info('Using "{}" as a close match to "{}" with a confidence of {}'.format(
                closest_match.text, statement.text, highest_confidence
            ))
        else:
            # If no closest match is found, return the default response
            return self.get_default_response(statement)

        recent_repeated_responses = filters.get_recent_repeated_responses(
            self.chatbot,
            statement.conversation
        )

        for index, recent_repeated_response in enumerate(recent_repeated_responses):
            self.chatbot.logger.info('{}. Excluding recent repeated response of "{}"'.format(
                index, recent_repeated_response
            ))

        response_selection_parameters = {
            'search_in_response_to': closest_match.search_text,
            'exclude_text': recent_repeated_responses,
            'exclude_text_words': self.excluded_words
        }

        if additional_response_selection_parameters:
            response_selection_parameters.update(additional_response_selection_parameters)

        # Get all statements that are in response to the closest match
        response_list = list(self.chatbot.storage.filter(**response_selection_parameters))

        if response_list:
            self.chatbot.logger.info(
                'Selecting response from {} optimal responses.'.format(
                    len(response_list)
                )
            )

            response = self.select_response(
                statement,
                response_list,
                self.chatbot.storage
            )

            response.confidence = highest_confidence
            self.chatbot.logger.info('Response selected. Using "{}"'.format(response.text))
        else:
            response = self.get_default_response(statement)

        return response

    def get_default_response(self, input_statement):
        del input_statement
        default_response = Statement(text="I'm sorry, I don't have an answer for that.")
        default_response.confidence = 0
        return default_response
