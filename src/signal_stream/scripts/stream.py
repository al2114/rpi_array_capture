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

CHUNK = 192000
RATE = 192000
CHANNELS = 2

NODE_ID = os.environ['ROS_NODE_ID']
IP_ADDR = os.environ['ROS_IP']

class SignalStreamer:

  def __init__(self):
    self.init_device()
    self.pub = rospy.Publisher("signal/data",StreamData,queue_size=4)

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
    current_time = rospy.get_rostime()
    offset = time_info['current_time']-time_info['input_buffer_adc_time']
    print((int(offset),int(1000000000*(offset-int(offset)))))
    offset_secs = int(offset)
    offset_nsecs = int(1000000000 * (offset - offset_secs))
    offset_duration = rospy.Duration(offset_secs, offset_nsecs)
    data = np.fromstring(in_data, dtype=np.int16)
    print((time_info, frame_count, status))
    msg = StreamData()
    msg.node_id = int(NODE_ID)
    msg.timestamp = current_time - offset_duration 
    msg.data = data
    self.pub.publish(msg)
    return (data, pyaudio.paContinue)

  def stop(self):
    self.stream.stop_stream()
    self.stream.close()
    self.p.terminate()


if __name__ == "__main__":
  streamer = SignalStreamer()
  rospy.init_node('signal_streamer', anonymous=True)
  rospy.loginfo("DSTL Node %s (%s) connected, ready to stream", NODE_ID, IP_ADDR) 
  try:
    streamer.start_stream()
  except KeyboardInterrupt:
    streamer.stop()
  rospy.spin()

