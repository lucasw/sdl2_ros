#!/usr/bin/env python
# generate a waveform

import rospy

from std_msgs.msg import Int16MultiArray


rospy.init_node("wave")

pub = rospy.Publisher("audio", Int16MultiArray, queue_size=1)
rospy.sleep(0.5)

msg = Int16MultiArray()

freq = 22050
duration = rospy.get_param("~duration", 1.0)
pwm = rospy.get_param("~pwm", 0.5)
wave_freq = rospy.get_param("~freq", 50)

wave_length = freq / wave_freq
rospy.loginfo("wave length " + str(wave_length))

vol = 2**15 - 1

count = 0
for i in range(int(freq * duration)):
    if count < pwm * wave_length:
        msg.data.append(-vol)
    else:
        msg.data.append(vol)
    count += 1
    if count == wave_length:
        count = 0

while not rospy.is_shutdown():
    pub.publish(msg)
    rospy.sleep(1.0 + duration)
