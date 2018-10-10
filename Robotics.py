# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy, cv2, numpy, actionlib
from geometry_msgs.msg import Twist,Point
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import Float32 ,String
from cv_bridge import CvBridge

class colour:
    def __init__(self):
        pass
class searchobject:
    # Four colours and the colours max and min values
    # Defining each of the colours
    red_mask = colour()
    yellow_mask = colour()
    blue_mask = colour()
    green_mask = colour()

    red_mask.min = numpy.array([0,30,81])
    red_mask.max = numpy.array([1,255,210])
    #redM = cv2.inRange(hsv,numpy.array((0,100,50)),numpy.array((5,255,255)))
    #red_mask = cv2.moments(redM)
    
    yellow_mask.min = numpy.array([29,30,81])
    yellow_mask.max = numpy.array([31,255,210])
    #yellowM = cv2.inRange(hsv,numpy.array((25,100,50)),numpy.array((30,255,255)))
    #yellow_mask = cv2.moments(yellowM)
    
    blue_mask.min = numpy.array([119,30,81])
    blue_mask.max = numpy.array([121,255,210])
    #blueM = cv2.inRange(hsv,numpy.array((110,100,50)),numpy.array((130,255,255)))
    #blue_mask = cv2.moments(blueM)
    
    green_mask.min = numpy.array([59,30,81])
    green_mask.max = numpy.array([61,255,210])
    #greenM = cv2.inRange(hsv,numpy.array((60,100,50)),numpy.array((70,255,255)))
    #green_mask = cv2.moments(greenM)
    
    red_mask.name = "red"
    yellow_mask.name = "yellow"
    blue_mask.name = "blue"
    green_mask.name = "green"
    
    red_mask.find = False
    yellow_mask.find = False
    blue_mask.find = False
    green_mask.find = False

    # The robot is set location to patrol
    # These are the x and y coordinates
    
    Patrol_X = [1,-1,-4.4,-3.5]
    Patrol_Y = [-3.3,3.3,0,2.5]
    
    # Current point
    i = 0 
    
    # Boolean value to show if the robot is at the location
    goal_Reached = False

	
    def __init__(self):
        cv2.namedWindow("Image", 1)
        cv2.namedWindow("Depth", 2)
        cv2.namedWindow("Mask", 3)
        self.bridge = CvBridge()
        cv2.startWindowThread()
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.update_image)
        self.depth_sub = rospy.Subscriber("/turtlebot/camera/depth/image_raw", Image, self.update_depth)
        self.twist_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist,queue_size=10)

    def begin(self):
	   
        # Keeping the while loop at within 10hz 
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # A exception error will be displayed if r.sleep variable get disturbed           
            r.sleep()

        # this section is where the robot moves to its assigned location (In patrol)
        # if robot has reached the location then it will move to the next one.
        # the robot will skip if the assigned location is already met
            while (self.goal_Reached!=True and self.i<len(self.Patrol_X)):
                self.goal_Reached = self.goto_goal(self.Patrol_X[self.i], self.Patrol_Y[self.i])
                if(self.goal_Reached == True):
                    self.i = self.i+1
            
            # searching for the objects
            self.movement()
            
            pole = False
            # if the colour of the object passes through
            # if the robot passes through the coloured object, the boolean value is true and goes to the if statement
            while (self.goto_obj(self.red_mask)):
                pole = True
                pass
            while (self.goto_obj(self.yellow_mask)):
                pole = True
                pass
            while (self.goto_obj(self.blue_mask)):
                pole = True
                pass
            while (self.goto_obj(self.green_mask)):
                pole = True
                pass
			
            if(pole):
                # Going back to previous location
                self.i = self.i-1
            self.goal_Reached = False
            # all the coloured object have been passed through, display output
            if(self.red_mask.find == True and self.yellow_mask.find == True and self.blue_mask.find == True and self.green_mask.find == True):
                rospy.loginfo("Task Completed")
				# if the task is not completed go back to the loop
                while (self.goal_Reached != True and self.i< len(self.Patrol_X)):
				    # goto next location, coloured object found
                    self.goal_Reached = self.goto_goal(self.Patrol_X[self.i], self.Patrol_Y[self.i])
                self.goal_Reached = False
				# finish the loop and go back to starting location
                while (self.goal_Reached != True):
                    # staring location coordinates 
                    self.goal_Reached = self.goto_goal(-4,-4.5)
				# Task completed, shut down the robot
                rospy.signal_shutdown( reason="Task Completed")


    def movement(self):
        boolean = True
        # The robot will rotate when a location has been reached and will search for the coloured objects
        rospy.loginfo("Searching for objects")
        i=0
        
        # Looping through when the boolean is true.
        # rotating around the location and searching
        while(boolean == True):
            turtle_base = Twist()
            turtle_base.linear.x = turtle_base.linear.y
            turtle_base.angular.z = 1
            self.twist_pub.publish(turtle_base)
            # if a coloured object has been seen by the robot
            # the found object code will be run            
            i = i+1
            if (self.found_obj(self.red_mask)):
                boolean = False
            if (self.found_obj(self.yellow_mask)):
                boolean = False
            if (self.found_obj(self.blue_mask)):
                boolean = False
            if (self.found_obj(self.green_mask)):
                boolean = False
            # if i is bigger than 300 then go to the next location
            if(i > 300):
                rospy.loginfo("Didn't see any objects, going to next position")
                boolean = False

    def found_obj(self,colour):
        # running the code to find the colour
        # if its already been found then it is true
        if(colour.find):
            pass
       
        else:
            # the colour range and the target image
            img_mask = cv2.inRange(self.hsv_img, colour.min, colour.max)
            # the size of the object            
            h, w = img_mask.shape
            # the size of the mask and it top and bottom ranges
            top = h / 4
            bottom = h - (h/4)
            # outside the range is black            
            img_mask[0:top, 0:w] = 0
            img_mask[bottom:h, 0:w] = 0
            
            # method to calculate the centre 
            if (img_mask.sum() > 100000):
                M = cv2.moments(img_mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # if the found objeect is less than 0.9 then the object has been found
                    if(self.depth_image[cy, cx] < 0.9):
                        return True
            else:
                return False


    def goto_obj(self,colour):
        bool_flag = False
        if(colour.find):
            pass
        else:
            img_mask = cv2.inRange(self.hsv_img, colour.min, colour.max)
            # size and ranges again
            h, w = img_mask.shape
            top = h / 4
            bottom = h - (h/4)
            img_mask[0:top, 0:w] = 0
            img_mask[bottom:h, 0:w] = 0
            if (img_mask.sum() > 0):
                M = cv2.moments(img_mask)
				
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # creating a circle for the robot to follow
                    cv2.circle(img_mask, (cx, cy), 20, (0, 0, 255), -1)
                    
                    if(self.depth_image[cy,cx]>0.08):
					# methods lines speed and the angular speed
                            # method twist                            
                            turtle_base = Twist()
                            turtle_base.linear.x = turtle_base.linear.y = turtle_base.angular.z = 0;
                            # calculating the different between the circle and the width of the screen.
                            # cx is the error                            
                            err = cx - w / 2
                            turtle_base.linear.x = 0.5
                            turtle_base.angular.z = -float(err) / 100
					# publish 	
                            self.twist_pub.publish(turtle_base)
				
                            bool_flag = True
					#if the size of the centre is less than 0.08 then run the loop
                    else:
				  # display the result   
                        rospy.loginfo("Found "+colour.name+", returning to Patrol position.")
                        colour.find = True
            cv2.imshow("Mask", img_mask)
            return bool_flag
            
    
#def Movin_robot(self, cx, w):
#        err = cx - w/2
#        self.twist.linear.x = 0.4
#        self.twist.angular.z = -float(err) / 100
#        self.cmd_vel_pub.publish(self.twist)
        

    def update_image(self, data):
        # converting the rgb to hsv
        bgr_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hsv_img = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("Image", bgr_image)

    def update_depth(self,data):
        # depth to rgb
        depth_data = self.bridge.imgmsg_to_cv2(data)
        depth_data.setflags(write=1)
        depth_data[numpy.isnan(depth_data)]=8
        self.depth_image=depth_data/8
        cv2.imshow("Depth", self.depth_image)

    def goto_goal(self, ax, ay):
        # setting the movement into coordinates 
        action = actionlib.SimpleActionClient("/turtlebot/move_base", MoveBaseAction)
	  
        # waiting 5 sec
        while (not action.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.sleep(5)
        goal = MoveBaseGoal()# setting the variable
		
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(ax, ay, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
		
        # output the infomation
        rospy.loginfo("Patrol location("+str(ax)+","+str(ay)+")")
        action.send_goal(goal)
        action.wait_for_result(rospy.Duration(120))

        if (action.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("Reached the location")
            return True
        else:
            rospy.loginfo("Failed to reach the location")
            return False


if __name__ == '__main__':
    rospy.init_node('searchobject')
    os=searchobject()
    os.begin()
    rospy.spin()
