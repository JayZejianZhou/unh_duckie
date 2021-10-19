class Move_motor(DTROS):
    """
    Maintained by Zejian@drz
    Move the Duckiebot's trajectory. This also serves as a ROS coding tempelate for the Duckiebot.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:
    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
    Subscribers:
    """
    
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LaneServoingNode, self).__init__(node_name=node_name,
                                               node_type=NodeType.LOCALIZATION)
        self.log("Initializing...")
        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")
        
        # set the left and right robot speed
        self.v, self.omega = 20, 30
        
        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        
    def publish_command(self, u):
        """Publishes a car command message.
        Args:
            u (:obj:`tuple(double, double)`): tuple containing [v, w] for the control action.
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = self.v  # v
        car_control_msg.omega = self.omega  # omega

        self.pub_car_cmd.publish(car_control_msg)

    def on_shutdown(self):
        self.loginfo("Stopping motors...")
        self.publish_command([0, 0])
        time.sleep(0.5)
        self.loginfo("Motors stopped.")

if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = Move_motor(node_name="motor_mover")
    # Keep it spinning
    rospy.spin()
    