import rospy
from std_msgs.msg import Float32MultiArray
from intera_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber



rospy.init_node('get_data')

def got_data(float32MultiArray, endpointState):
    print float32MultiArray.data[0]
    print endpointState.pose.position.x

pcf_sub = Subscriber("/sensor_values", Float32MultiArray)
ft_sub = Subscriber("/robot/limb/right/endpoint_state", EndpointState)
# pcf_sub = Subscriber("/robot/joint_states", JointState)

ats = ApproximateTimeSynchronizer([pcf_sub, ft_sub], queue_size=5,  slop=0.01, allow_headerless=True)
ats.registerCallback(got_data)
rospy.spin()