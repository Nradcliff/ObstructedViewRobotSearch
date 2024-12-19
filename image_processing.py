import rospy
from std_msgs.msg import String

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.command_pub = rospy.Publisher('robot_commands', String, queue_size=10)
        self.feedback_sub = rospy.Subscriber('robot_feedback', String, self.feedback_callback)
        self.feedback = ""

    def feedback_callback(self, msg):
        self.feedback = msg.data
        rospy.loginfo(f"Received feedback: {self.feedback}")

    def send_command(self, command):
        try:
            self.command_pub.publish(command)
            rospy.loginfo(f"Sent command: {command}")
        except Exception as e:
            rospy.logerr(f"Failed to send command: {e}")

    def move_robot(self, direction):
        command = f"Move {direction}"
        self.send_command(command)

    def turn_robot(self, direction):
        command = f"Turn {direction}"
        self.send_command(command)

    def stop_robot(self):
        command = "Stop"
        self.send_command(command)

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
