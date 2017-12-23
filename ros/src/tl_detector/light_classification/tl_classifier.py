from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import rospy
import time
import os
import sys
import tensorflow as tf
from io import StringIO
from light_classification.tf_dependencies import label_map_util
from collections import defaultdict


class TLClassifier(object):
    def __init__(self):
        # Setting Paths for Model and Labels
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        model = curr_dir + '/tf_dependencies/frozen_inference_graph.pb'
        labels = curr_dir + '/tf_dependencies/label_map.pbtxt'
        label_map = label_map_util.load_labelmap(labels)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=14,
                                                                    use_display_name=True)
        # Based on Tensorflow Object Detection API
        self.category_index = label_map_util.create_category_index(categories)
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)
        self.current_light = TrafficLight.UNKNOWN


    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            min_score = 0.5
            img_expanded = np.expand_dims(img, axis=0)
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: img_expanded})

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            for i in range(boxes.shape[0]):
                if scores[i] > min_score:
                    class_name = self.category_index[classes[i]]['name']
                    if class_name == 'Green':
                        rospy.loginfo('Color Detected: Green')
                        return TrafficLight.GREEN
                    elif class_name == 'Yellow':
                        rospy.loginfo('Color Detected: Yellow')
                        return TrafficLight.YELLOW
                    elif class_name == 'Red':
                        rospy.loginfo('Color Detected: Red')
                        return TrafficLight.RED
                else:
                    return TrafficLight.UNKNOWN
                    rospy.loginfo('Color Detected: Unknown')
