import rospy
import actionlib
import math, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.35
LIDAR_ERROR = 0.015
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
PI=math.pi

twist = Twist()
roll=pitch=yaw = 0.0
is_stop = False

def get_scan():
    scan = rospy.wait_for_message('scan', LaserScan)
    scan_filter = []
    
    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
    samples_view = 1            # 1 <= samples_view <= samples

    if samples_view > samples:
        samples_view = samples

    if samples_view is 1:
        scan_filter.append(scan.ranges[0])

    else:
        left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
        right_lidar_samples_ranges = samples_view//2
        
        left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
        right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
        scan_filter.extend(left_lidar_samples + right_lidar_samples)

    for i in range(samples_view):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0
    
    return scan_filter


#Ref:https://www.youtube.com/watch?v=2Qs6A3Zsbfk

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

def get_is_stop(msg):
    global is_stop
    if abs(msg.twist.twist.angular.z) < 0.001:
        is_stop = True
    else:
        is_stop = False 

def rotate(theta):
    # theta range: 0~180: CCW rotation, -180~0: CW rotation
    # Converting from angles to radians
    print 'rotate s'
    start_yaw = yaw
    objective_rad = yaw+theta*PI/180
    
    if (objective_rad>PI):
	    target = objective_rad - 2*PI
    elif (objective_rad<(-1*PI)):
	    target = objective_rad + 2*PI
    else:
	    target = objective_rad
	
    # We wont use linear components
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.x = 0
    twist.angular.y = 0
    
    while(1):
        twist.angular.z = target-yaw
        if 0<twist.angular.z<0.1:
            twist.angular.z = 0.1
        elif -0.1<twist.angular.z<0:
            twist.angular.z=-0.1
        

        pub.publish(twist)

        if(abs(target-yaw)<0.1):
            twist.angular.z = 0.0
            pub.publish(twist)
            break
    
    print 'rotate e'

def movebase_client(x,y):
    print 'movebase_client s'
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0
    # Sends the goal to the action server.
    client.send_goal(goal)
    #time.sleep(2)

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    print 'movebase_client e'
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()


def g_localization():
    try:
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        angle=[180,90,180,-90]
        for i in angle:
            print 'Go forward'
            twist.linear.x = 0.22
            t0 = time.time()
            t1 = time.time()
            while((t1-t0)<5): # max time = 5
                pub.publish(twist)
                t1 = time.time()
                lidar_distances = get_scan()
                min_distance = min(lidar_distances)
                if min_distance < SAFE_STOP_DISTANCE:
                    break

            print 'Stop!!'
            twist.linear.x = 0
            pub.publish(twist)
            print 'Rotate {}'.format(i)
            rotate(i)

        
    except:
	    rospy.logwarn("global_localization service is not available")
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('initializing')
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/odom', Odometry, get_rotation)
    stop_sub = rospy.Subscriber('/odom', Odometry, get_is_stop)
    print 'Wait please...'
    time.sleep(2)
    print 'Starting soon'
    time.sleep(1)

    try:
        rospy.wait_for_service("global_localization",timeout=2.0)
        GL=rospy.ServiceProxy("global_localization",Empty)
        MC=rospy.ServiceProxy("/move_base/clear_costmaps",Empty)
        GL()
        g_localization()
        # result = movebase_client(6.4,-2.5)
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        pass
    
    while not is_stop:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
    MC()
    print 'End initialize'