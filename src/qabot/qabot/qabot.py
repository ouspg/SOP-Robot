from rclpy.node import Node
import os
import rclpy
from std_msgs.msg import String
from haystack.components.retrievers.in_memory import InMemoryBM25Retriever
from haystack.components.retrievers import SentenceWindowRetriever
from haystack.components.converters import TextFileToDocument
from haystack.components.preprocessors import DocumentCleaner, DocumentSplitter
from haystack.components.writers import DocumentWriter
from haystack.components.joiners import DocumentJoiner
from pathlib import Path
from haystack.document_stores.in_memory import InMemoryDocumentStore
from haystack.utils import ComponentDevice, Device
from haystack import Pipeline


file_paths = [
    "src/qabot/resource/data" / Path(name)
    for name in os.listdir("src/qabot/resource/data")
]


def index_documents():
    document_store = InMemoryDocumentStore()
    text_file_converter = TextFileToDocument()
    document_joiner = DocumentJoiner()
    document_cleaner = DocumentCleaner()
    document_splitter = DocumentSplitter(
        split_by="word", split_length=150, split_overlap=50
    )

    document_writer = DocumentWriter(document_store)
    preprocessing_pipeline = Pipeline()
    preprocessing_pipeline.add_component(
        instance=text_file_converter, name="text_file_converter"
    )
    preprocessing_pipeline.add_component(
        instance=document_joiner, name="document_joiner"
    )
    preprocessing_pipeline.add_component(
        instance=document_cleaner, name="document_cleaner"
    )
    preprocessing_pipeline.add_component(
        instance=document_splitter, name="document_splitter"
    )
    preprocessing_pipeline.add_component(
        instance=document_writer, name="document_writer"
    )
    preprocessing_pipeline.connect("text_file_converter", "document_joiner")
    preprocessing_pipeline.connect("document_joiner", "document_cleaner")
    preprocessing_pipeline.connect("document_cleaner", "document_splitter")
    preprocessing_pipeline.connect("document_splitter", "document_writer")

    preprocessing_pipeline.run({"text_file_converter": {"sources": file_paths}})
    return document_store


class QaBotClientNode(Node):
    def __init__(self):
        super().__init__("qabot_node")
        self.publisher_ = self.create_publisher(String, "chatbot_response", 10)
        self.subscription = self.create_subscription(
            String, "recognized_speech", self.tts_callback, 10
        )
        self.get_logger().info("Starting!")
        self.device = ComponentDevice.from_single(Device.gpu(0))

        document_store = index_documents()
        bm25_retriever = InMemoryBM25Retriever(
            document_store=document_store, scale_score=True
        )

        sentence_window_retriever = SentenceWindowRetriever(
            document_store=document_store, window_size=1
        )
        self.pipe = Pipeline()
        self.pipe.add_component("bm25_retriever", bm25_retriever)
        self.pipe.add_component("sentence_window_retriever", sentence_window_retriever)
        self.pipe.connect(
            "bm25_retriever", "sentence_window_retriever.retrieved_documents"
        )

    def tts_callback(self, msg):
        data = self.chatbot_worker_callback((msg.data).lower())
        if data:
            tts_msg = String()
            tts_msg.data = data
            self.publisher_.publish(tts_msg)
        else:
            pass

    def chatbot_worker_callback(self, query):
        self.get_logger().info(query)
        greetings = [
            "terve",
            "hei",
            "päivää",
            "moi",
            "iltaa",
            "huomenta",
            "moikka",
        ]
        if query in greetings:
            return "terve"
        pred = self.pipe.run(
            data={"bm25_retriever": {"query": query, "top_k": 1}},
            include_outputs_from={"bm25_retriever"},
        )
        if pred["bm25_retriever"]["documents"][0].score > 0.3:
            final_response_text = pred["bm25_retriever"]["documents"][0].content
        else:
            retrieved_documents = [
                doc.content for doc in pred["sentence_window_retriever"]["documents"]
            ]
            final_response_text = " ".join(retrieved_documents)

        self.get_logger().info(final_response_text)
        self.publisher_.publish(String(data=final_response_text))


def main():
    rclpy.init()
    qaclient = QaBotClientNode()
    rclpy.spin(qaclient)
    qaclient.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
