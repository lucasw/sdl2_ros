#!/usr/bin/env python
# generate a waveform

import math
import random
import rospy

from std_msgs.msg import Int16MultiArray


rospy.init_node("wave")

pub = rospy.Publisher("audio", Int16MultiArray, queue_size=1)
rospy.sleep(0.5)

msg = Int16MultiArray()

freq = 22050
duration = rospy.get_param("~duration", 2.5)
pwm = rospy.get_param("~pwm", 0.5)
wave_freq = rospy.get_param("~freq", 50)

wave_length = freq / wave_freq
rospy.loginfo("wave length " + str(wave_length))

vol = 2**15 - 1

count = 0
wave_count = 0
for i in range(int(freq * duration)):
    val = 0
    if False:
        val = vol
        if count < pwm * wave_length:
            val = -vol
    if True:
        noise_vol = 5000
        noise = random.random() * noise_vol
        if wave_length > 0:
            val = (vol - noise_vol) * math.sin(2.0 * math.pi * count / wave_length) + noise

    msg.data.append(val)
    count += 1
    if count == wave_length:
        count = 0
        sweep = True
        wave_count += 1
        if sweep and wave_count % 18 == 0:
            wave_length = int(wave_length * 2.0 / 3.0)  # - 10  # *= 2.0 / 3.0
            print wave_count, wave_length

while not rospy.is_shutdown():
    pub.publish(msg)
    rospy.sleep(1.0 + duration)
