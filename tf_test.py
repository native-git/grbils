#! /usr/bin/env python

import tf
import math
import rospy
from std_srvs.srv import Empty

quat = [0,0,0.703,0.711]

euler = tf.transformations.euler_from_quaternion(quat)

rad = euler[2]
degrees = math.degrees(euler[2])

d1 = math.degrees(1.558620080414757)
d2 = math.degrees(1.5639785596783606)

print d1
print d2

print abs(d1-d2)