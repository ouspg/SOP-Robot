#!/usr/bin/env python

import separate
import argparse
import struct
import time, datetime
import os
import rospy
import wave
from audio import Audio
from multiprocessing import Process
from std_msgs.msg import Int16

def speech_segregation(args):

    while True:
        # check if any audio inputs have been captured
        try:
            wav = wave.open(Audio.WAVE_OUTPUT_FILENAME, 'r')
            wav.close()
            # if file is found, append it to wave_scp with timestamp as key
            dt = datetime.datetime.now()
            timestamp = "%04d%02d%02d%02d%02d%02d" % (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
            new_filename = "wave{}.wav".format(timestamp)
            os.rename(Audio.WAVE_OUTPUT_FILENAME, new_filename)
            # write file information to separator in format: 
            # ID filename
            with open(args.wave_scp, 'a') as scp:
                scp.write("{} {}\n".format(timestamp, new_filename))

            separationProcess = Process(target=separate.run, args=(args,))
            separationProcess.start()
        except Exception:
            time.sleep(1)
            continue


def main():
    pubDOA = rospy.Publisher('DOA', Int16)
    # prevRecordingStart = datetime.datetime.now()
    audio = Audio()

    rospy.init_node('speech', anonymous=True)

    rate = rospy.Rate(10) #10 Hz
    
    while not rospy.is_shutdown():
        voiceActive = audio.readVAD()

        if voiceActive:
            # publish direction to ROS topic when voice is active
            direction = Int16(data=audio.readDOA())
            pubDOA.publish(direction)

            # start recording in new process so direction publication can continue without waiting for the recording to end
            recordingProcess = Process(target=audio.record)
            recordingProcess.start()

        rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=
        "Command to run speech separation ROS node"
    )
    parser.add_argument(
        "--config", type=str, help="Location of training configure files",
        default="tune/train.yaml")
    parser.add_argument(
        "--state_dict", type=str, help="Location of networks state file",
        default="tune/epoch.19.pkl") # HOX pkl file is too large to be saved in git, request from author or create new
    parser.add_argument(
        "--wave_scp",
        type=str,
        help="Location of input wave scripts in kaldi format",
        default="wave.scp")
    parser.add_argument(
        "--cuda",
        default=False,
        action="store_true",
        dest="cuda",
        help="If true, inference on GPUs")
    parser.add_argument(
        "--dump-dir",
        type=str,
        dest="dump_dir",
        default="cache",
        help="Location to dump seperated speakers")
    parser.add_argument(
        "--dump-mask",
        default=False,
        action="store_true",
        dest="dump_mask",
        help="If true, dump mask matrix")
    args = parser.parse_args()

    # listen inputs in separate thread
    process = Process(target=speech_segregation, args=(args,))
    process.start()
    # run main in current thread
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        process.terminate()