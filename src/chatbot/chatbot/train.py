#!/usr/bin/python3

from chatbot import chatbot
from chatterbot.trainers import ListTrainer
from chatterbot.trainers import ChatterBotCorpusTrainer

trainer = ChatterBotCorpusTrainer(chatbot)
#trainer = ListTrainer(chatbot)

trainer.train(
    ".src/chatbot/chatbot/train_stanford"
)

