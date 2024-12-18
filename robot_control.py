import rospy
from std_msgs.msg import String

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.command_pub = rospy.Publisher('robot_commands', String, queue_size=10)
        self.battery_sub = rospy.Subscriber('battery_status', String, self.battery_callback)
        self.obstacle_sub = rospy.Subscriber('obstacle_detection', String, self.obstacle_callback)
        self.battery_status = ""
        self.obstacle_detected = False
        self.state = "stopped"

    def battery_callback(self, msg):
        self.battery_status = msg.data
        rospy.loginfo(f"Battery status: {self.battery_status}")

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data == "obstacle"
        if self.obstacle_detected:
            rospy.logwarn("Obstacle detected! Stopping the robot.")
            self.stop_robot()

    def send_command(self, command):
        self.command_pub.publish(command)
        rospy.loginfo(f"Sent command: {command}")

    def move_robot(self, direction):
        if not self.obstacle_detected:
            command = f"Move {direction}"
            self.send_command(command)
            self.state = "moving"
        else:
            rospy.logwarn("Cannot move, obstacle detected.")

    def stop_robot(self):
        command = "Stop"
        self.send_command(command)
        self.state = "stopped"

    def turn_robot(self, direction):
        if not self.obstacle_detected:
            command = f"Turn {direction}"
            self.send_command(command)
            self.state = "turning"
        else:
            rospy.logwarn("Cannot turn, obstacle detected.")

    def command_loop(self):
        while not rospy.is_shutdown():
            command = input("Enter command (move/turn/stop): ")
            if command.startswith("move"):
                direction = command.split()[1] if len(command.split()) > 1 else "forward"
                self.move_robot(direction)
            elif command.startswith("turn"):
                direction = command.split()[1] if len(command.split()) > 1 else "left"
                self.turn_robot(direction)
            elif command == "stop":
                self.stop_robot()
            elif command == "exit":
                rospy.loginfo("Exiting command loop.")
                break
            else:
                rospy.logwarn("Unknown command. Please use 'move', 'turn', or 'stop'.")

if __name__ == "__main__":
    controller = RobotController()
    controller.command_loop()
