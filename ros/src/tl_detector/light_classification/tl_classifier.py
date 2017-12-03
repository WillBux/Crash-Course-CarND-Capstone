from styx_msgs.msg import TrafficLight
from keras.models import load_model
from keras.preprocessing import image
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = load_model("model.h5")
        pass

    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        img = cv2.resize(img, (600, 800))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        pred = self.model.predict(x)
        if pred:
            return TrafficLight.Green
        return TrafficLight.Red
