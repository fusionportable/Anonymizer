"""
Utils for rosbag file processing

"""
import rosbag
from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys
from std_msgs.msg import Header
import numpy as np

del sys.path[1]
import cv2


def load_bag_msg(bag: rosbag.Bag, topic: str):
      """
      Load a ROS bag with image topic to an image iterator.

      Args: 
            bag: rosbag.Bag: the bag to load
            topic: str: the topic to load from the bag

      Returns:
            iterator: iterator: the iterator for the topic
            total_frames: int: the total number of frames in the topic
      """

      # get the total number of frames to write
      total_frames = bag.get_message_count(topic_filters=topic)
      print(f"Total frames: {total_frames} in {topic}")
      # get an iterator for the topic with the frame data
      iterator = bag.read_messages(topics=topic)

      return iterator, total_frames


def bag_to_images(bag: rosbag.Bag, output_file: str, topic: str):
      """
      Convert a ROS bag with image topic to images with timestamp as names.
      (With a txt timestamp file)      
      Args:
            bag: the bag file to get image data from
            output_file: the path to a directory to write images to
            topic: the topic to read image data from
      
      Returns:
            output_file: the path to the directory with the images
      
      """
      # create a bridge to convert ROS messages to OpenCV images
      bridge = CvBridge()
      # get the total number of frames to write
      total_frames = bag.get_message_count(topic_filters=topic)
      print(f"Total frames: {total_frames} in {topic}")
      # get an iterator for the topic with the frame data
      iterator = bag.read_messages(topics=topic)
      # check if the output directory exists
      if not os.path.isdir(output_file):
            # create the output directory
            os.makedirs(output_file)
      # iterate over the image messages of the given topic
      timestamp = os.path.join(output_file, 'timestamp.txt')
      i = 0
      f =  open(timestamp, 'w')
      for _, msg, _ in tqdm(iterator, total=total_frames):
            # read the image data into a NumPy tensor
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            timestamp = "%.6f" % msg.header.stamp.to_sec()
            f.write(str(i) + ' ' + timestamp + '\n')
            name = timestamp + '.png'
            # img = get_camera_image(msg.data, (msg.height, msg.width))
            # write the image to the video file
            cv2.imwrite(os.path.join(output_file, name), cv_image)
            i += 1
      f.close()

      return output_file


def extract_topic_to_bag(bag_in: rosbag.Bag, topic: str, bag_out: rosbag.Bag):
      """
      Extract a topic from a ROS bag to a new bag file.

      Args:
            bag: rosbag.Bag: the bag to extract from
            topic: str: the topic to extract
            output_bag: rosbag.Bag: the bag to write to

      Returns:
            None
      """
      # load msg iterator
      topic_msg, topic_frames = load_bag_msg(bag_in, topic)
      # loop through the iterator
      for _, msg, _ in tqdm(topic_msg, total=topic_frames):
            bag_out.write(topic, msg, msg.header.stamp)

      bag_in.close()
      bag_out.close()


def extract_topics_to_bag(bag_in: rosbag.Bag, topics: list, bag_out: rosbag.Bag):
      """
      Extract a list of topics from a ROS bag to a new bag file.

      Args:
            bag: rosbag.Bag: the bag to extract from
            topics: list: the topics to extract
            output_bag: rosbag.Bag: the bag to write to

      Returns:
            None
      """
      if not isinstance(topics, list):
            topics = [topics]

      # process each topics
      print(f"Extracting {len(topics)} topics in {bag_in.filename}")

      # load msg iterator
      for topic in topics:
            print("====================================")
            print(f"=====Extracting {topic}: =======")
            topic_msg, topic_frames = load_bag_msg(bag_in, topic)
            print(f"Total frames: {topic_frames} in {topic}")
            for _, msg, _ in tqdm(topic_msg, total=topic_frames):
                  bag_out.write(topic, msg, msg.header.stamp)
            print("{topic} is extracted")

      # close the bags
      bag_in.close()
      bag_out.close()


##### Utils for ros msg conversion
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
       
def convert_ros_img_to_img(ros_img, encoding = 'bgr8', compressed = True):
      bridge = CvBridge()
      img = None
      if compressed:
            if encoding == 'mono16':
                  img = bridge.compressed_imgmsg_to_cv2(ros_img, encoding)
            elif encoding == 'bgr8':
                  img = bridge.compressed_imgmsg_to_cv2(ros_img, 'bgr8')
      else:
            if encoding == 'mono16':
                  img = bridge.imgmsg_to_cv2(ros_img, encoding)
            elif encoding == 'bgr8':
                  img = bridge.imgmsg_to_cv2(ros_img, encoding)
      return img

