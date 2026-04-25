from pathlib import Path
import importlib
import os
import random

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        document_stores = importlib.import_module("haystack.document_stores")
        standard_pipelines = importlib.import_module(
            "haystack.pipelines.standard_pipelines"
        )
        haystack_nodes = importlib.import_module("haystack.nodes")
        haystack_pipelines = importlib.import_module("haystack.pipelines")

        InMemoryDocumentStore = document_stores.InMemoryDocumentStore
        TextIndexingPipeline = standard_pipelines.TextIndexingPipeline
        BM25Retriever = haystack_nodes.BM25Retriever
        FARMReader = haystack_nodes.FARMReader
        ExtractiveQAPipeline = haystack_pipelines.ExtractiveQAPipeline
        self.document_store = InMemoryDocumentStore(use_bm25=True)
        self.doc_dir = Path(get_package_share_directory("qabot")) / "resource" / "data"
        self.files_to_index = [str(self.doc_dir / filename) for filename in os.listdir(self.doc_dir)]
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
