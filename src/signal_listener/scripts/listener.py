#!/usr/bin/env python3

from signal_stream.msg import StreamData
import numpy as np
import rospy 
import os
import scipy.io

NODE_ID = os.environ['ROS_NODE_ID']
IP_ADDR = os.environ['ROS_IP']
N_NODES = 2
THREE_SAMPS = rospy.Duration(0,15625) # 3/192000

class SignalListener:

  epochs = [None] * N_NODES
  chunks = [0] * N_NODES
  data = [np.array([])] * N_NODES

  def __init__(self):
    self.sub = rospy.Subscriber("signal/data", StreamData, self.callback, queue_size=100)
      
  def callback(self, msg):
    nid = msg.node_id
    if self.epochs[nid] is None:
      self.epochs[nid] = msg.timestamp
    self.chunks[nid] = self.chunks[nid]+1
    time_signature = "[%d.%d]" % (msg.timestamp.secs, msg.timestamp.nsecs)
    # Combine real and imaginary parts
    print("%s Received chunk %d from node %d" % (time_signature, self.chunks[nid], nid))
    data = [complex(d[0],d[1]) for d in np.array(msg.data).reshape(int(len(msg.data)/2),2)]
    self.data[nid] = np.append(self.data[nid],data)
    #print(time_signature + " Received from node " + str(msg.node_id)) 
    print('[{:.0f},{:.0f}, ... , {:.0f},{:.0f}]'.format(data[0],data[1],data[-2],data[-1]))    

  def deinit(self):
    self.dump_data()

  def time_from_framecount(self, nsamples):
    return THREE_SAMPS * nsamples/3

  def dump_data(self):
    for epoch in self.epochs:
      if epoch is None:
        return
      print(epoch)
    begin = max(self.epochs)
    data_lengths = [len(d) for d in self.data]
    frame_offsets = [int(3*(begin-epoch)/THREE_SAMPS) for epoch in self.epochs]
    window_length = min([(frames-offset) for (frames,offset) in zip(data_lengths,frame_offsets)])
    if window_length is None:
      return
    print(window_length)
    data_stripped = np.array([self.data[i][frame_offsets[i]:frame_offsets[i]+window_length] for i in range(N_NODES)])
    #print(data_stripped.shape)
    #np.savetxt("data_dump.csv",data_stripped,fmt=('%.0f%+.0fi,'*N_NODES)[:-1])
    scipy.io.savemat('data_dump.mat',dict(X=data_stripped))


if __name__ == "__main__":
  listener = SignalListener()
  rospy.init_node('signal_listener')
  rospy.loginfo("DSTL Signal Listener active, listening on node %s (%s)", NODE_ID, IP_ADDR)
  rospy.spin()
  rospy.on_shutdown(listener.deinit)
