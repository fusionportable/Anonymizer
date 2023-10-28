"""A script to convert a ROS bag image topic to images with timestamp as names."""

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


def images_to_bag(input_images: str, reference_bag: rosbag.Bag, output_bag: rosbag.Bag, topic = '/stereo/frame_left/image_raw/compressed', image_list = 'timestamp.txt'):
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
      # assert os.path.isfile(reference_bag), "Reference bag file does not exist!"
      # assert topic in reference_bag.get_type_and_topic_info()[1].keys(), "Topic does not exist in reference bag!"

      timestamp_list = []
      order = open(image_list, 'r')
      for _, _, t in reference_bag.read_messages(topics=[topic]):
            # print(t.to_sec())
            # timestamp = "%.6f" % t.to_sec()
            timestamp_list.append(t.to_sec())
      reference_bag.close()
	
      # images = glob.glob(os.path.join(input_images, '*.png'))
      # sorted(images, key=lambda x: os.path.basename(x).strip('.png'))
      # images = images.sort()
      # print('Processing images: {}'.format(len(images)))
      # print('Processing anonymized images: {}'.format(len(timestamp_list)))
      # print(images)
	
      for timestamp in tqdm(timestamp_list, total=len(timestamp_list)):
            if rospy.is_shutdown():
                  break
            line = order.readline()
            line = line.strip('\n')
            image_name = line.split(' ')[-1]
            header = Header(stamp=rospy.Time.from_sec(timestamp), frame_id='frame_left')
            cv_image = cv2.imread(os.path.join(input_images, image_name + '.png'))
            img_msg = convert_img_to_ros_img(cv_image, 'bgr8', header, compressed=True)
            output_bag.write(topic, img_msg, img_msg.header.stamp)

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


if __name__ == '__main__':
      parser = argparse.ArgumentParser(description='Convert a series of images to a ROS bag with specified image topic (if available).')
      parser.add_argument('--input_images', type=str, help='the path to a directory to read images from')
      parser.add_argument('--reference_bag', type=str, help='the bag file to get reference timestamp from')
      parser.add_argument('--output_bag', type=str, help='the bag file to write image data to')
      parser.add_argument('--topic', type=str, help='the topic to write image data to')
      parser.add_argument('--image_list', type=str, help='the timestamp order to load image data from')
      args = parser.parse_args()
      # open the bag file
      assert os.path.isfile(args.reference_bag), "Reference bag file does not exist!"
      reference_bag = rosbag.Bag(args.reference_bag)
      output_bag = rosbag.Bag(args.output_bag, 'w')
      # convert the bag file to images
      images_to_bag(args.input_images, reference_bag, output_bag, args.topic, args.image_list)
      # close the bag file
      output_bag.close()