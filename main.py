import time
import rospy
from image_processing import ImageProcessor
from llm_processing import LLMProcessor
from robot_control import RobotController

def main():
    image_processor = ImageProcessor()
    llm_processor = LLMProcessor()
    robot_controller = RobotController()

    # Initialize variables
    text_prompts = ["abstract object", "target object"]
    image_path = "path/to/image.jpg"  # Placeholder for static image path
    camera_active = False  # Flag to indicate if camera is active

    while not rospy.is_shutdown():
        try:
            # Step 1: Capture image from camera or use static image
            if camera_active:
                image_path = image_processor.capture_image()  # Assuming capture_image() method exists
            else:
                # Use static image for testing
                print(f"Using static image: {image_path}")

            # Step 2: Detect objects
            probs = image_processor.detect_objects(image_path, text_prompts)
            labels = image_processor.get_object_labels(probs)
            print("Detection probabilities:", probs)
            print("Object labels:", labels)

            # Step 3: Generate text prompt based on detection
            prompt = "Where can I find the target object?"
            response = llm_processor.get_response(prompt)
            print("LLM Response:", response)

            # Step 4: Send command to robot
            robot_controller.move_robot(response)

            # Step 5: Get feedback from the robot
            feedback = robot_controller.get_feedback()  # Assuming get_feedback() method exists
            print("Robot Feedback:", feedback)

            # Optional: Adjust prompts based on feedback
            if "not found" in feedback:
                prompt = "Please provide more specific directions."
                response = llm_processor.get_response(prompt)
                robot_controller.move_robot(response)

            time.sleep(1)  # Loop delay

        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    rospy.init_node('robot_controller_node')  # Initialize the ROS node
    main()
