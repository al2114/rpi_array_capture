#!/usr/bin/env python3

from signal_stream.msg import StreamData
import numpy as np
import time
import rospy
import serial
import pyaudio
import sys
import os 
import struct
import scipy.io

CHUNK = 192000
RATE = 192000
CHANNELS = 2

NODE_ID = os.environ['ROS_NODE_ID']
IP_ADDR = os.environ['ROS_IP']

class SignalStreamer:

  data = np.array([])

  def __init__(self):
    self.init_device()

  def init_device(self):
    self.p = pyaudio.PyAudio()
    device_found = False
    attempts = 1
    print("Searching for audio devices")
    while not device_found and attempts < 5:
      for i in range(self.p.get_device_count()):
        info = self.p.get_device_info_by_index(i)
        if "FUNcube" in info["name"]:
          print("Found FUNcube Dongle on index " + str(info["index"]) + ", registering device info")
          self.device_info = info
          return
      # Else device has not been found
      print("FUNcube Dongle has not been found, searching again in 5 seconds...")
      time.sleep(5)
      attempts += 1
    print("Cannot find FUNcube Dongle, exiting")
    sys.exit()

  def start_stream(self):
    self.stream = self.p.open(format=pyaudio.paInt16,
                         channels=CHANNELS,
                         rate=RATE,
                         input=True,
                         frames_per_buffer=CHUNK,
                         input_device_index=self.device_info["index"],
                         stream_callback=self.callback)
    self.stream.start_stream()

  def callback(self, in_data, frame_count, time_info, status):
    data = np.fromstring(in_data, dtype=np.int16)
    complex_data = [complex(d[0],d[1]) for d in data.reshape(int(len(data)/2),2)]
    self.data = np.append(self.data,complex_data)
    return (data, pyaudio.paContinue)

  def stop(self):
    print("Finishing")
    #print(self.data[0:500])
    scipy.io.savemat('record_dump.mat',dict(x=self.data))
    self.stream.stop_stream()
    self.stream.close()
    self.p.terminate()


if __name__ == "__main__":
  streamer = SignalStreamer()
  rospy.init_node('signal_streamer', anonymous=True)
  rospy.loginfo("DSTL Node %s (%s) connected, ready to stream", NODE_ID, IP_ADDR) 
  streamer.start_stream()
  rospy.spin()
  rospy.on_shutdown(streamer.stop)

