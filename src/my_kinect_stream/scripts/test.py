import tensorflow as tf
from tensorflow import keras
# from tensorflow.keras.preprocessing import image
from keras.preprocessing import image
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

# Load the model from the desktop
model = keras.models.load_model('/home/raghad/turtlebot2_ws/src/my_kinect_stream/keras_model.h5')

# Load the class labels from the desktop
with open('/home/raghad/turtlebot2_ws/src/my_kinect_stream/labels.txt', 'r') as file:
    class_labels = [line.strip() for line in file]

# Initialize the camera subscriber
class ImageClassifier:
    def init(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess the frame for prediction
            frame = cv2.resize(frame, (224, 224))
            img_array = image.img_to_array(frame)
            img_array = np.expand_dims(img_array, axis=0)
            img_array = img_array / 255.0  # Normalize the image data

            # Make a prediction using the loaded model
            predictions = model.predict(img_array)

            # Get the class with the highest probability
            predicted_class_index = np.argmax(predictions[0])
            predicted_class = class_labels[predicted_class_index]

            # Print the predicted class
            print(f"Predicted class: {predicted_class}")

        except Exception as e:
            print(f"Error processing image: {e}")

def main():
    rospy.init_node('image_classifier_node', anonymous=True)
    image_classifier = ImageClassifier()
    rospy.spin()

if __name__ == "main":
    main()