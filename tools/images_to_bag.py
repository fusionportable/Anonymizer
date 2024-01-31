import argparse
import rosbag
from tqdm import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys
import os
import numpy as np
import glob
import rospy
from std_msgs.msg import Header

del sys.path[1]
import cv2


def images_to_bag(input_images: str, reference_bag: rosbag.Bag, output_bag: rosbag.Bag, topic: str, image_list: str):
      """
      Convert a series of images to a ROS bag with specified image topic (if available).
      
      Args:
            input_images: the path to a directory to read images from
            reference_bag: the bag file to get reference timestamp from
            output_bag: the bag file to write image data to
            topic: the topic to write image data to
      
      Returns:
            None
      
      """
      # sanity check
      assert os.path.isdir(input_images), "Input images directory does not exist!"

      total_frames = reference_bag.get_message_count(topic_filters=topic)
      iterator = reference_bag.read_messages(topics=topic)
      order =  open(image_list, 'r')
      
      for _, msg, _ in tqdm(iterator, total = total_frames):
            if rospy.is_shutdown():
                  break
            image_name = order.readline().strip('\n').split(' ')[-1]
            cv_image = cv2.imread(os.path.join(input_images, image_name))
            img_msg = convert_img_to_ros_img(cv_image, 'bgr8', msg.header, compressed=True)
            output_bag.write(topic, img_msg, img_msg.header.stamp)
      
      order.close()
      
      print("Convert Done!")


##### Sensor data
def convert_img_to_ros_img(image, encoding, header, compressed=False):
	bridge = CvBridge()
	img_msg = None
	if not compressed:
		if encoding == 'mono16':
			image_16UC1 = image.astype(np.uint16)[:, :, 0]
			img_msg = bridge.cv2_to_imgmsg(image_16UC1, encoding=encoding, header=header)
		elif encoding == 'bgr8':
			img_msg = bridge.cv2_to_imgmsg(image, encoding=encoding, header=header)
		return img_msg
	else:
		if encoding == 'mono16':
			image_16UC1 = image.astype(np.uint16)[:, :, 0]
			img_msg = bridge.cv2_to_compressed_imgmsg(image_16UC1)
		elif encoding == 'bgr8':
			img_msg = bridge.cv2_to_compressed_imgmsg(image)
		img_msg.header = header
		return img_msg


def parser_dual():
      parser = argparse.ArgumentParser()
      parser.add_argument('--left')
      parser.add_argument('--right')
      parser.add_argument('--reference')
      parser.add_argument('--output')
      parser.add_argument('--left_topic')
      parser.add_argument('--right_topic')
      parser.add_argument('--left_list')
      parser.add_argument('--right_list')
      
      return parser.parse_args()
      

def parser():
      parser = argparse.ArgumentParser()
      parser.add_argument('--input')
      parser.add_argument('--reference')
      parser.add_argument('--output')
      parser.add_argument('--topic')
      parser.add_argument('--image_list')
      
      return parser.parse_args()


def main():
      args = parser()
      reference_bag = rosbag.Bag(args.reference)
      output_bag = rosbag.Bag(args.output, 'w')
      images_to_bag(args.input, reference_bag, output_bag, args.topic, args.image_list)
      output_bag.close()
      
      
def main_dual():
      args = parser_dual()
      reference_bag = rosbag.Bag(args.reference)
      output_bag = rosbag.Bag(args.output, 'w')
      images_to_bag(args.left, reference_bag, output_bag, args.left_topic, args.left_list)
      rospy.sleep(1)
      images_to_bag(args.right, reference_bag, output_bag, args.right_topic, args.right_list)
      


if __name__ == "__main__":
      main()