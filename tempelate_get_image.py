#!/usr/bin/env python3

import os
import cv2
import yaml
import time
import rospy
import numpy as np

from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

#import visual_servoing_activity
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.ros import compressed_imgmsg_to_rgb, rgb_to_compressed_imgmsg


class LaneServoingNode(DTROS):
    """
    Performs a form of visual servoing based on estimates of the image-space lane orientation
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node
    Configuration:
    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
    Subscribers:
        ~/image/compressed (:obj:`CompressedImage`):
            compressed image
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LaneServoingNode, self).__init__(node_name=node_name,
                                               node_type=NodeType.LOCALIZATION)
        self.log("Initializing... step 1")
        
        ############ Put your own definition code  #####################
        # set the name of the robot
        self.veh = "drz2"


        ############ End Put your own definition code  #####################

        # Size of the image
        w, h = 640, 480
        self._cutoff = ((int(0.5 * h), int(0.01 * h)), (int(0.1 * w), int(0.1 * w)))

        self.VLS_ACTION = None
        self.VLS_STOPPED = True

        # Defining subscribers:
        rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.cb_image,
            #buff_size=10000000,
            buff_size = 5,
            queue_size=1
        )
        print("subscribed image topic: " + f"/{self.veh}/camera_node/image/compressed")
        
	# select the current activity
        rospy.Subscriber(f"/{self.veh}/vls_node/action", String, self.cb_action, queue_size=1)

        # Command publisher
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic, Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self._lt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_control/left_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        self._rt_mask_pub = rospy.Publisher(
            f"/{self.veh}/visual_control/right_mask/image/compressed",
            CompressedImage,
            queue_size=1
        )

        # Get the steering gain (omega_max) from the calibration file
        # It defines the maximum omega used to scale normalized steering command
        kinematics_calib = self.read_params_from_calibration_file()
        self.omega_max = kinematics_calib.get('omega_max', 6.0)

        #self.log("this step is completted12")

        for _ in range(5):
            self.log("Initializing... zz steps")
            time.sleep(1)
        self.log("Initialized!")
        #self.log("Waiting for the Exercise App \"Visual Lane Servoing\" to be opened in VNC...")


    def cb_action(self, msg):
        """
        Call the right functions according to desktop icon the parameter.
        """

        if msg.data not in ["init", "calibration", "go", "stop"]:
            self.log(f"Activity '{msg.data}' not recognized. Exiting...")
            exit(1)

        self.VLS_ACTION = msg.data

        self.loginfo(f"ACTION: {self.VLS_ACTION}")

        if not self.AIDO_eval:
            if self.VLS_ACTION == "init":
                self.log("Put the robot in a lane. Press [Calibrate] when done.")
                return

            if self.VLS_ACTION == "calibration":
                self.log("Using your hands if you are working with a real robot, or the joystick if "
                         "you are working with the simulator. Turn the robot (in place), to the left "
                         "then to the right by about 30deg on each side. Press [Go] when done.")
                return
            if self.VLS_ACTION == "go":
                self.log(f"Calibration value: {int(self.steer_max)}")
                self.VLS_STOPPED = False
                # NOTE: this is needed to trigger the agent and get another image back
                self.publish_command([0, 0])
                return

            if self.VLS_ACTION == "stop":
                self.publish_command([0, 0])
                self.VLS_STOPPED = True
                return

    def detect_lane_markings(self, image):
        """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
        """
    
    	######################### Put your own lane detection code ########################
    	# sample code for testing, delete it
        h, w, _ = image.shape
        mask_left_edge = np.random.rand(h, w)
        mask_right_edge = np.random.rand(h, w)
    
        return (mask_left_edge, mask_right_edge)
        

    def cb_image(self, image_msg):
        """
        Processes the incoming image messages.
        Performs the following steps for each incoming image:
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the
        image that doesn't include the road
        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message
        """
        print('image recieved')
        image = compressed_imgmsg_to_rgb(image_msg)

        # Resize the image to the desired dimensionsS
        height_original, width_original = image.shape[0:2]
        img_size = image.shape[0:2]
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, tuple(reversed(img_size)), interpolation=cv2.INTER_NEAREST)

        # crop image
        (top, bottom), (left, right) = self._cutoff
        image = image[top:-bottom, left:-right, :]


        if self.is_shutdown:
            self.publish_command([0, 0])
            return

        shape = image.shape[0:2]

        (lt_mask, rt_mask) = self.detect_lane_markings(image)

        lt_mask_viz = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                      lt_mask.astype(np.uint8), 0.8, 0)
        rt_mask_viz = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0.1,
                                      rt_mask.astype(np.uint8), 0.8, 0)

        lt_mask_viz = rgb_to_compressed_imgmsg(cv2.cvtColor(lt_mask_viz, cv2.COLOR_GRAY2RGB), "jpeg")
        rt_mask_viz = rgb_to_compressed_imgmsg(cv2.cvtColor(rt_mask_viz, cv2.COLOR_GRAY2RGB), "jpeg")

        self._lt_mask_pub.publish(lt_mask_viz)
        self._rt_mask_pub.publish(rt_mask_viz)


        ########### write the control code ################
        ########### output:  u=[w, v]

        #sample function, delete it
        u=[0.2, 0.2]






        #################################################
        if self.VLS_ACTION == "calibration":
            self.steer_max = max(self.steer_max,
                                 2 * max(float(np.sum(lt_mask * steer_matrix_left_lm)),
                                         float(np.sum(rt_mask * steer_matrix_right_lm))))

        if self.VLS_ACTION != "go" or self.VLS_STOPPED:
            return

        if self.steer_max == -1:
            self.logerr("Not Calibrated!")
            return

        steer = float(np.sum(lt_mask * steer_matrix_left_lm)) + \
                float(np.sum(rt_mask * steer_matrix_right_lm))

        # now rescale from 0 to 1
        steer_scaled = np.sign(steer) * \
                       rescale(min(np.abs(steer), self.steer_max), 0, self.steer_max)

        # u = [self.v_0, steer_scaled * self.omega_max]
        self.publish_command(u)

        # self.logging to screen for debugging purposes
        self.log("    VISUAL SERVOING    ")
        self.log(f"Steering: (Unnormalized) : {int(steer)} / {int(self.steer_max)},"
                 f"  Steering (Normalized) : {np.round(steer_scaled, 1)}")
        self.log(f"Command v : {np.round(u[0], 2)},  omega : {np.round(u[1], 2)}")

    def publish_command(self, u):
        """Publishes a car command message.
        Args:
            u (:obj:`tuple(double, double)`): tuple containing [v, w] for the control action.
        """

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v

        car_control_msg.omega = u[1]  # omega

        self.pub_car_cmd.publish(car_control_msg)

    @staticmethod
    def trim(value, low, high):
        """
        Trims a value to be between some bounds.
        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound
        Returns:
            the trimmed value
        """
        return max(min(value, high), low)

    def angle_clamp(self, theta):
        if theta > 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjusts the ROS parameters for the
        node with the new values.
        """

        def readFile(fname):
            with open(fname, "r") as in_file:
                try:
                    return yaml.load(in_file, Loader=yaml.FullLoader)
                except yaml.YAMLError as exc:
                    self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                    return None

        # Check file existence
        cali_file_folder = "/data/config/calibrations/kinematics/"
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            fname = cali_file_folder + "default.yaml"
            self.logwarn("Kinematic calibration %s not found! Using default instead." % fname)
            return readFile(fname)
        else:
            return readFile(fname)

    def on_shutdown(self):
        self.loginfo("Stopping motors...")
        self.publish_command([0, 0])
        time.sleep(0.5)
        self.loginfo("Motors stopped.")


def rescale(a: float, L: float, U: float):
    if np.allclose(L, U):
        return 0.0
    return (a - L) / (U - L)


if __name__ == "__main__":
    # Initialize the node
    encoder_pose_node = LaneServoingNode(node_name="visual_lane_servoing_node")
    # Keep it spinning
    rospy.spin()



