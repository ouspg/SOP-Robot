#!/usr/bin/python3

from chatterbot import ChatBot
import logging
from chatterbot.trainers import ChatterBotCorpusTrainer

logging.basicConfig(level=logging.DEBUG)


chatbot = ChatBot("Helper",
                    read_only=True,
                    logic_adapters=[
                            {
                            #"chatterbot.logic.BestMatch"
                            "import_path": "best_match_adapter.BestMatchAdapter",
                            'default_response': 'Paihoittelut, mutta en ymmärtänyt kysymystä',
                            'maximum_similarity_threshold': 0.5
                            }]


                    )

trainer = ChatterBotCorpusTrainer(chatbot)

trainer.train(
    "./src/chatbot/chatbot/train_stanford"
    
)

while True:
    try:
        bot_input = chatbot.get_response(input())
        print(bot_input)

    except(KeyboardInterrupt, EOFError, SystemExit):
        break
