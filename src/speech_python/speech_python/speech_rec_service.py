import rclpy
from rclpy.node import Node
import json

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import speech_recognition as sr

from msg_interface.srv import SpeechRec

class SpeechRecServer(Node):

    def __init__(self):
        super().__init__('speech_rec_server')
        self.response = ""
        self.server = self.create_service(SpeechRec, "speech_rec_server", self.callback_speech_rec)
        self.get_logger().info("SpeechRecServer started...")

    def callback_speech_rec(self, request, response):
        #self.r = sr.Recognizer()
        response = self.response
        response.transcript = {"Pläp"}
        response.confidence = {0.69}
        return response
        self.devices = {}
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            self.devices[name] = index
            self.get_logger().warn("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))
        for name, index in self.devices.items():
            try:
                self.source = sr.Microphone(device_index=index, sample_rate=16000)
                self.get_logger().warn("Set microphone with name {1} and device_index={0} as source.".format(index, name))
                break
            except Exception as e:
                self.get_logger().warn(f"Did not manage to get connection with device {name} with index {index}")

        with self.source as source:

            #self.get_logger().info("Say something!")
            #audio = self.r.listen(source)
            
            try:
                # for testing purposes, we're just using the default API key
                # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                #self.get_logger().warn("speech_recognition output:\n{}".format(self.r.recognize_google(audio, language="fi")))
                # instead of `r.recognize_google(audio)`
                #data = self.r.recognize_google(audio, language="fi")
                #self.get_logger().info(f"{type(data)}, {data[0]}")
                #self.recognized_data =
                response.transcript = {"Pläp"}
                response.confidence = {0.69}
                #response.confidence = data['alternative'][0]['confidence']
                #self.all_data = self.r.recognize_google(audio, language="fi", show_all=True)

                return response
                #self.get_logger().info("Google Speech Recognition thinks you said " + self.recognized_data)
            except sr.UnknownValueError:
                self.get_logger().info("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                self.get_logger().info("Could not request results from Google Speech Recognition service; {0}".format(e))     

#def main(args=None):
#    rclpy.init(args=args)
#    node = SpeechRecServer()
#    rclpy.spin(node)
#    rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    try:
        service_server = SpeechRecServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(service_server)
        try:
            executor.spin()
        except:
            pass
        finally:
            executor.shutdown()
            service_server.destroy_node()
    except:
        pass
    finally:
        rclpy.shutdown()
if __name__ == '__main__':
    main()