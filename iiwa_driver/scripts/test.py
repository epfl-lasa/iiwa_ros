import rospy
from std_msgs.msg import Float64MultiArray
# from iiwa_tools.srv import GetIK
import tf

rospy.init_node('test')

pub = rospy.Publisher('/iiwa/PassiveDS/command', Float64MultiArray, queue_size=10)

cmd_msg = Float64MultiArray()

# eef_target = [0, 0, 0, 0.45, 0., 1.19]
eef_target = [0, 0, 0, 0.7, 0.14, 0.577]
vel = [0, 0, 0, 0, 0, 0]

# rospy.wait_for_service('/iiwa/iiwa_ik_server')
# ik_client = rospy.ServiceProxy('/iiwa/iiwa_ik_server', GetIK)

listener = tf.TransformListener()

while not rospy.is_shutdown():
    dt = 0.001

    try:
        (trans, rot) = listener.lookupTransform('/world', '/iiwa_link_ee', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    # print msg.position
    for i in range(3):
        vel[i+3] = 10*(eef_target[i+3] - trans[i])
    print(vel)
    cmd_msg.data = vel
    pub.publish(cmd_msg)

    rospy.sleep(dt)