from chatbot import chatbot
from chatterbot.trainers import ListTrainer
from chatterbot.trainers import ChatterBotCorpusTrainer

trainer = ChatterBotCorpusTrainer(chatbot)
#trainer = ListTrainer(chatbot)

trainer.train(
    "data"
)

