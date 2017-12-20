from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import rospy
try:
    from keras.models import load_model
    from keras.preprocessing import image
except ImportError:
    rospy.logwarn("KERAS could not be imported!")

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        try:
            self.model = load_model("model.h5")
        except:
            rospy.logwarn("Classifier model could not be loaded")
        pass

    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        try:
            img = cv2.resize(img, (600, 800))
            x = image.img_to_array(img)
            x = np.expand_dims(x, axis=0)
            pred = self.model.predict(x)
            color = np.argmax(pred)
            if color == 2:
                return TrafficLight.Green
            if color == 1:
                return TrafficLight.Yellow
            if color == 2:
                return TrafficLight.Red
            else:
                return TrafficLight.UNKNOWN
        except:
            return TrafficLight.UNKNOWN
