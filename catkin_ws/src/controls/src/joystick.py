# bypass the planner entirely and directly publish to the topic controls-force-blah-blah and PID topics
# LOOK at rosjoy and jstest packages

import keyboard 
import rospy
from sensor_msgs.msg import Joy

class Controller:
    # get keyboard inputs
    # def __init__(self):
        # self.joy = Joy()
        # self.joy.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        # self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        # self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        # self.rate = rospy.Rate(10)

    def execute(self):
        while not rospy.is_shutdown():
            # self.joy_pub.publish(self.joy)
            # self.rate.sleep()

            if keyboard.is_pressed('w'):
                self.joy.axes[1] = 1
                # positive surge 
            elif keyboard.is_pressed('s'):
                self.joy.axes[1] = -1
                # negative surge
            else:
                self.joy.axes[1] = 0

            if keyboard.is_pressed('a'):
                self.joy.axes[0] = 1
                # positive sway
            elif keyboard.is_pressed('d'):
                self.joy.axes[0] = -1
                # negative sway
            else:
                self.joy.axes[0] = 0
            
            if keyboard.is_pressed('q'):
                self.joy.axes[3] = 1
                # positive yaw
            elif keyboard.is_pressed('e'):
                self.joy.axes[3] = -1
                # negative yaw
            else:
                self.joy.axes[3] = 0

            if keyboard.is_pressed('up'):
                self.joy.axes[5] = 1
            elif keyboard.is_pressed('down'):
                self.joy.axes[5] = -1
            else:
                self.joy.axes[5] = 0

            if keyboard.is_pressed('left'):
                self.joy.axes[4] = 1
            elif keyboard.is_pressed('right'):
                self.joy.axes[4] = -1
            else:
                self.joy.axes[4] = 0

            if keyboard.is_pressed('space'):
                self.joy.buttons[0] = 1
                # global z up
            else:
                self.joy.buttons[0] = 0
                
            if keyboard.is_pressed('shift'):
                self.joy.buttons[1] = 1
                # global z down
            else:
                self.joy.buttons[1] = 0

            if keyboard.is_pressed('ctrl'):
                self.joy.buttons[2] = 1
            else:
                self.joy.buttons[2] = 0
            if keyboard.is_pressed('z'):
                self.joy.buttons[3] = 1
            else:
                self.joy.buttons[3] = 0
            if keyboard.is_pressed('x'):
                self.joy.buttons[4] = 1
            else:
                self.joy.buttons[4] = 0
            if keyboard.is_pressed('c'):
                self.joy.buttons[5] = 1
            else:
                self.joy.buttons[5] = 0
            if keyboard.is_pressed('v'):
                self.joy.buttons[6] = 1
            else:
                self.joy.buttons[6] = 0
            if keyboard.is_pressed('b'):
                self.joy.buttons[7] = 1
            else:
                self.joy.buttons[7] = 0

            # if we want to implement heave could we do the same thin
            # as we are doing with surge and other sway (might need to 
            # deactivate all position pid loops when moving then) maybe
            # could set a default thruster value of the value of the 
            # pid when it was shut off or could constalty update the 
            # goal of z pid to be current z when moving (some way of 
            # stopping the bot from just dropping to the bottom when 
            # moving with this system) 
                
            if self.joy.axes[0] == 0 and self.joy.axes[1] == 0:
                self.joy.axes[0] = 0
                self.joy.axes[1] = 0
                # run pid loop to make bot stay in position
            
            if self.joy.axes[3] == 0:
                self.joy.axes[3] = 0
                # run pid loop to make bot stay in rotation

                
if __name__ == "__main__":
    rospy.init_node("controller")
    controller = Controller()
    controller.execute()
    rospy.spin()


# TOPIC NAMES: 
# 1. self.surge = Superimposer.Degree_Of_Freedom('/controls/force/surge')
# self.sway = Superimposer.Degree_Of_Freedom('/controls/force/sway')
# self.heave = Superimposer.Degree_Of_Freedom('/controls/force/heave')
# self.roll = Superimposer.Degree_Of_Freedom('/controls/torque/roll')
# self.pitch = Superimposer.Degree_Of_Freedom('/controls/torque/pitch')
# self.yaw = Superimposer.Degree_Of_Freedom('/controls/torque/yaw') **January 16, 2024 6:46 PM**

# 1. /controls/pid/x/setpoint
# 2. /controls/pid/y/setpoint
# 3. /controls/pid/z/setpoint

# CONTROLLER STUFF
# +’s and -’s might need to be changed 
# 4 goals 
# x goal, y goal, z goal, quaternion goal for pid 
# or x force, y force, z force, rotational force vector 

# CURRENT SOLUTION!!!
# map axis 1 to pid setpoint that is a specific distance away (or surge) 
# map axis 0 to sway
# button 1 and 2 to global z (pid loop) 
# axis 3 to yaw 
# when both axis is 0 set a pid loop to maintain that position, deactivate when bot is moving 
# and deactivate the loop when it is not 0 also deactivate the loop for rotational pids to stay at 0 when yaw is not 0 then set them again when yaw is 0
# if we want to implement heave could we do the same thing as we are doing with surge and other sway (might need to deactivate all position pid loops when moving then) maybe could set a default thruster value of the value of the pid when it was shut off or could constalty update the goal of z pid to be current z when moving (some way of stopping the bot from just dropping to the bottom when moving with this system) 

# T1 = Axis1 + Axis3
# T2 = Axis1 - Axis3
# T3 = -Axis3 + Axis0 
# T4 = -Axis3 - Axis0
# T5 = Controller().move([None, None, height]) height += increment when Axis5 is pressed and height -= increment when Button5 is pressed - Axis4 + Button1 - Button2
# T7 = Controller().move([None, None, height]) height += increment when Axis5 is pressed and height -= increment when Button5 is pressed + Axis4 - Button1 + Button2
# T6 = Controller().move([None, None, height]) height += increment when Axis5 is pressed and height -= increment when Button5 is pressed - Axis4 - Button1 + Button2
# T8 = Controller().move([None, None, height]) height += increment when Axis5 is pressed and height -= increment when Button5 is pressed + Axis4 + Button1 - Button2

# ResetRotation Button3 = Controller.RotateEuler(0, 0, 0)
# test using efforts vs pid for movement 
# use function that converts global to local 

# DESIGN A CONTROL SCHEME 
# 1. look at the bot and the field and the competition scoring and rules 
# 2. add weights so it is neutrally buoyant like scuba divers do  
# 3. stop effort mode and pid from fighting
# 4. have they looked at drone software? 
# 5. merge x and y pid? 
# 6. have they considered doing up a up down pid and a forwards AND sideways combined PID
# 7. could they do like an xdrive set up with the sub (like how can drones fly in all directions at once) 
# 8. RL System
#     1. completing tasks gives a reward equal to the reward in the competition (maybe points for reaching a task aswell) 
#     2. eventually it would randomly stumble upon an obstacle and get points, it would learn to do this more and more and more and more, until it got really good, we would need to make sure the simulation was randomized, so the rl system is adaptive
#     3. takes in sensor input as input (ei position, depth, camera)
#     4. outputs a sway sure heave pitch roll and yaw (and other stuff like fire torpedo etc)