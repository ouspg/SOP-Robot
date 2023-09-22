#!/usr/bin/python3

import spacy
from chatterbot.logic import LogicAdapter
from chatterbot import filters
from chatterbot.conversation import Statement
from sklearn.metrics.pairwise import cosine_similarity
from chatterbot.conversation import Statement
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

    def process(self, input_statement, additional_response_selection_parameters=None):
        input_vec = self.nlp(input_statement.text).vector

        closest_match = None
        highest_confidence = 0
        max_statements = 100
        
        for statement in self.chatbot.storage.filter(in_response_to=None):

            statement_vec = self.nlp(statement.text).vector

            input_vec_normalized = input_vec / np.linalg.norm(input_vec)
            statement_vec_normalized = statement_vec / np.linalg.norm(statement_vec)
            confidence = cosine_similarity([input_vec_normalized], [statement_vec_normalized])[0][0]

            if confidence > highest_confidence:
                highest_confidence = confidence
                closest_match = statement

                if confidence >= 0.95:
                    break

        if closest_match is not None:
            self.chatbot.logger.info('Using "{}" as a close match to "{}" with a confidence of {}'.format(
                closest_match.text, input_statement.text, highest_confidence
            ))
        else:
            # If no closest match is found, return the default response
            return self.get_default_response(input_statement)


        self.chatbot.logger.info('Using "{}" as a close match to "{}" with a confidence of {}'.format(
            closest_match.text, input_statement.text, highest_confidence
        ))

        recent_repeated_responses = filters.get_recent_repeated_responses(
            self.chatbot,
            input_statement.conversation
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
                input_statement,
                response_list,
                self.chatbot.storage
            )

            response.confidence = highest_confidence
            self.chatbot.logger.info('Response selected. Using "{}"'.format(response.text))
        else:
            response = self.get_default_response(input_statement)

        return response

    def get_default_response(self, input_statement):
        default_response = Statement(text="I'm sorry, I don't have an answer for that.")
        default_response.confidence = 0
        return default_response