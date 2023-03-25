#!/usr/bin/env python3

from chatterbot import ChatBot
import logging
from chatterbot.trainers import ChatterBotCorpusTrainer

logging.basicConfig(level=logging.DEBUG)


chatbot = ChatBot("Helper",
                    read_only=True,
                    logic_adapters=["chatterbot.logic.BestMatch"]
                    )

trainer = ChatterBotCorpusTrainer(chatbot)

trainer.train(
    "/workspace/src/chatbot/chatbot/trainingdata"
    
)

while True:
    try:
        bot_input = chatbot.get_response(input())
        print(bot_input)

    except(KeyboardInterrupt, EOFError, SystemExit):
        break
