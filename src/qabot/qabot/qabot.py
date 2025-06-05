from rclpy.service import Service
from rclpy.action import ActionClient
from rclpy.node import Node

import rclpy
import random
from std_msgs.msg import String


#import logging

from haystack.document_stores import InMemoryDocumentStore
import os
from haystack.pipelines.standard_pipelines import TextIndexingPipeline
from haystack.nodes import BM25Retriever
from haystack.nodes import FARMReader
from haystack.pipelines import ExtractiveQAPipeline

from pprint import pprint
from haystack.utils import print_answers

#logging.basicConfig(level=logging.INFO)

class QaBotClientNode(Node):

    def __init__(self):
        super().__init__('qabot_node')
        self.publisher_ = self.create_publisher(
            String, 
            'chatbot_response', 
            10)
        self.subscription = self.create_subscription(
            String, 
            'recognized_speech', 
            self.tts_callback, 
            10)
        self.get_logger().info('Starting!')
        self.document_store = InMemoryDocumentStore(use_bm25=True)
        self.doc_dir = "./src/qabot/resource/data/"
        self.files_to_index = [self.doc_dir + "/" + f for f in os.listdir(self.doc_dir)]
        self.indexing_pipeline = TextIndexingPipeline(self.document_store)
        self.indexing_pipeline.run_batch(file_paths=self.files_to_index)
        self.retriever = BM25Retriever(document_store=self.document_store)
        self.reader = FARMReader(model_name_or_path="timpal0l/mdeberta-v3-base-squad2", use_gpu=True)
        self.pipe = ExtractiveQAPipeline(self.reader, self.retriever)
        self.greetings = ["terve", "hei", "päivää", "moi", "iltaa", "huomenta", "moikka"]

    def tts_callback(self, msg):
        data = self.chatbot_worker_callback((msg.data).lower())
        if(data):
            tts_msg = String()
            tts_msg.data = data
            self.publisher_.publish(tts_msg)
        else:
            pass
        
    def chatbot_worker_callback(self, data):
        self.get_logger().info(data)
        if(data == "hei"):
            return("terve")
        pred = self.pipe.run(
            query=data, params={"Retriever": {"top_k": 10}, "Reader": {"top_k":5}}
        )
        if(pred['answers'][0].score < 0.3):
            #final_response_text = "Anteeksi, en tiedä vastausta"
            final_response_text = random.choice(self.greetings)
            #self.get_logger().info("Retrieved score was %s for answer %s" % (pred['answers'][0].score, pred['answers'[0].answer]))
        else:
            final_response_text = pred['answers'][0].answer
            self.get_logger().info("Found answer: %s with score %s" % ((final_response_text), pred['answers'][0].score))
        return(final_response_text)

def main():
    rclpy.init()
    qaclient = QaBotClientNode()
    rclpy.spin(qaclient)
    qaclient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
