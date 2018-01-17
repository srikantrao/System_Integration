#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from math import sqrt

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):

        # Initialize the TL Detector Node
        rospy.init_node('tl_detector')
        
        # Initialize   
        self.pose = None
        self.waypoints = []
        self.camera_image = None
        self.lights = []

        ## Adding additional variables that might be required here
        self.position = None    # position in free space in point form
        self.orientation = None # orientation in free space in quarternion form
        self.theta = None # Yaw angle
        
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # Not needed at this stage - will be needed when using the classifier 
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
		# For now there is no classifier setup
        #self.light_classifier = TLClassifier()
        #self.listener = tf.TransformListener()


        #Publish the /traffic_waypoint --> std_msgs/Int32
        self.loop()

        # TODO: Still need to implement the classifer based system which needs to respond to image_cb


    def loop(self):
        rate = rospy.Rate(10)
        #rate = rospy.Rate(self.config['tl']['detector_rate'])
        while not rospy.is_shutdown():
            # Get closest waypoint and traffic state
            light_wp, state = self.process_traffic_lights()

            # Update the last states
            self.last_state = self.state
            self.state = state

            # Update the WayPoint Index
            self.last_wp = light_wp

            # Publish the message based on whether the light is red or not
            # template borrowed from image_cb method (should only be used for simulation)
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                if state == TrafficLight.GREEN and light_wp is not None:
                    light_wp = -light_wp
                elif state == TrafficLight.UNKNOWN:
                    light_wp = -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

            rospy.logwarn("Closest Waypoint Index is: %d", self.last_wp)

            rate.sleep()

    def pose_cb(self, msg):
        """ Get parameters from the /current_pose topic """
        self.pose = msg.pose

        #rospy.logwarn("Pose is : %s.", self.pose)

        # Get hold of the vehicle's current position
        self.position = msg.pose.position
        # Get hold of the vehicle's current orientation
        self.orientation = msg.pose.orientation

        # Quaternion to theta angle transformation
        euler = tf.transformations.euler_from_quaternion([
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w])

        # Extract Yaw
        self.theta = euler[2]

    def waypoints_cb(self, msg):
        # Consists of a header and a list of waypoints -- Make a copy and then unsubscribe
        # Make a copy of all the waypoints from /base_waypoints
        for waypoint in msg.waypoints:
            self.waypoints.append(waypoint)

        # Unregister from this waypoint as the information is only needed once.
        self.sub2.unregister()


    def traffic_cb(self, msg):
        """Used to capture the status of the lights. Msg contains an array of TrafficLights
        provides information on 1. Current Status 2. Pose (Position and Header)
        """

        for light in msg.lights:
            self.lights.append(light)

        #rospy.logwarn("Traffic Lights have been found: %s", self.lights)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        Why is there the link to the Wikipedia article ? It is simply finding the closest point to pose ?
        """
        closestDistance = float("inf")
        closestWayPointIndex = -1
        for index, waypoint in enumerate(self.waypoints):
            distance = self.euclideanDistance(pose.position, waypoint.pose.pose.position)
            if distance < closestDistance:
                closestDistance = distance
                closestWayPointIndex = index

        return closestWayPointIndex

    def euclideanDistance(self, point1, point2):
        """
        Measures the Euclidean Distance between point1 and point2
        :param point1: Derived from Point ( x, y and z although z is not useful in  this case)
        :param point2: Derived from Point ( x, y and z although z is not useful in this case)
        :return: float value of the euclidean Distance
        """
        return sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def get_light_state(self, light):
        """Determines the current color of the traffic light. Needs to be implemented for the classifier

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def find_closest_light(self, pose):
        """
        Similar to get_closest_waypoint to find the closest traffic light
        :param pose(Pose): Will use pose.position
        :return: Return index based on traffic lights array that is published
        """
        closestDistance = float("inf")
        closestLightIndex = -1
        for index, light in enumerate(self.lights):
            distance = self.euclideanDistance(pose.position, light.pose.pose.position)
            if distance < closestDistance:
                closestDistance = distance
                closestLightIndex = index

        return closestLightIndex

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection

        if(self.pose is not None):

            stop_line_positions = self.config['stop_line_positions']

            # Find the closest waypoint
            car_position = self.get_closest_waypoint(self.pose)

            #rospy.logwarn("[TL_DETECTOR] Closest WP id: %s.", car_position)

            # Find the closest traffic light
            lightIndex = self.find_closest_light(self.pose)

            #rospy.logwarn("[TL_DETECTOR] Closest TL id: %s.", lightIndex)

            #If a valid lightIndex has been found, determine location, current_distance and color
            if lightIndex >= 0:

                # Get the traffic light and its current state (using this only for simulator currently)
                trafficLight = self.lights[lightIndex]
                trafficState = trafficLight.state

                # Get the global x and y position of the line to stop at for the traffic light
                stopLineLocation = self.config['stop_line_positions'][lightIndex]

                # Get the global x and y positions of the traffic light
                lightLocation = (trafficLight.pose.pose.position.x, trafficLight.pose.pose.position.y)

                # Need to create dummy Pose object
                stopLinePose = Pose()
                stopLinePose.position.x = stopLineLocation[0]
                stopLinePose.position.y = stopLineLocation[1]



                # Get the closest waypoint ID to the upcoming stop line
                waypointID = self.get_closest_waypoint(stopLinePose)

                # Get the distance between car and the Traffic Light Stop Line
                distance = self.euclideanDistance(self.pose.position, stopLinePose.position)

                # For Debugging Purposes
                #rospy.logwarn("TL Index: %d, State: %d, Position: %.2f, %.2f, %.2f", lightIndex, trafficState,
                              #stopLineLocation[0], stopLineLocation[1], distance)

                # As pointed out earlier , return -1 if none exists

                if waypointID < 0:
                    return -1, TrafficLight.UNKNOWN

                #rospy.logwarn("TL Distance: %.2f, Closest Waypoint Index: %d", distance, waypointID)

                return waypointID, trafficState

            else:
                rospy.logwarn("TL state was -1. Need to debug why this happened ")
                return -1, TrafficLight.UNKNOWN
        else:
            rospy.logwarn("Pose of the car is unknown right now!!")
            return -1, TrafficLight.UNKNOWN

        # Something else goes wrong and it has not returned a position yet
        rospy.logwarn("Some unknown error occured. It was not able to find WPID and TL index")
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
