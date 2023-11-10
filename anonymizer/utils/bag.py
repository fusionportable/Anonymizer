"""
      Utils for rosbag file processing

"""
import sys
sys.path.append('../anonymizer')
import rosbag
from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys
from std_msgs.msg import Header
import numpy as np
from itertools import zip_longest

from anonymizer.bin import anonymize_bag

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


def in_interval(abs_time:float, time: float, intervals: list):
      """
            Args:
                  time: float: the time to check
                  intervals: list: the list of intervals
            Returns:
                  bool: if the time is in the intervals
      """
      for interval in intervals:
            if time >= abs_time + interval[0] and time <= abs_time + interval[1]:
                  return True
      return False


def ifmask(abs_time:float, time: float):
      """
            Args:
                  time: float: the time to check
                  intervals: list: the list of intervals
            Returns:
                  bool: if the time is in the intervals
      """
      if time >= abs_time and time <= abs_time + 10:
            return True
      
      return False


def complete_frame(bag_raw: rosbag.Bag, bag_anonymized: rosbag.Bag, bag_out: rosbag.Bag, weights_path: str):
      right_topic = '/stereo/frame_right/image_raw/compressed'
      iterator_raw, frames_raw = load_bag_msg(bag_raw, right_topic)
      iterator_anonymized, frames_anonymized = load_bag_msg(bag_anonymized, right_topic)

       # anonymizer parameters
      face_threshold = 0.2
      plate_threshold = 0.2
      obfuscation_parameters = '21,2,9'
      # init anonymizer
      anonymizer, detection_threshold = anonymize_bag.init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters)

      print("Processing right frame topic")
      for raw, anonymized in tqdm(zip_longest(iterator_raw, iterator_anonymized), total=frames_raw):
            if anonymized is not None:
                  _, msg, _ = anonymized
                  bag_out.write(right_topic, msg, msg.header.stamp)
            else:
                  _, msg, _ = raw
                  image = convert_ros_img_to_img(msg, 'bgr8', True)
                  anonymized_image, _ = anonymizer.anonymize_image(image, detection_threshold)
                  ros_image = convert_img_to_ros_img(anonymized_image, 'bgr8', msg.header, True)
                  bag_out.write(right_topic, ros_image, ros_image.header.stamp)
      
      print("Processing left frame topic")
      left_topic = '/stereo/frame_left/image_raw/compressed'
      iterator_anonymized, frames_anonymized = load_bag_msg(bag_anonymized, left_topic)

      for left in tqdm(iterator_anonymized, total=frames_anonymized):
            _, msg, _ = left
            bag_out.write(left_topic, msg, msg.header.stamp)
      
      print("close the bag for memory save")
      bag_raw.close()
      bag_anonymized.close()
      bag_out.close()
      print("Done")



def bag_merge(bag0: rosbag.Bag, bag1: rosbag.Bag, bag_out: rosbag.Bag, intervals: list, weights_path: str):
      """
            Args:
                  bag0: raw bag with unobfuscated data
                  bag1: obfuscated bag with obfuscated data (defected bag)
                  bag_out: output bag with obfuscated data (refined bag)
      """

      left_topic = '/stereo/vehicle_frame_left/image_raw/compressed'
      right_topic = '/stereo/vehicle_frame_right/image_raw/compressed'
      iterator0_left, left_frames0 = load_bag_msg(bag0, left_topic)
      iterator0_right, right_frames0 = load_bag_msg(bag0, right_topic)
      iterator1_left, left_frames1 = load_bag_msg(bag1, left_topic)
      iterator1_right, right_frames1 = load_bag_msg(bag1, right_topic)
      
      # anonymizer parameters
      face_threshold = 0.2
      plate_threshold = 0.2
      obfuscation_parameters = '21,2,9'
      # init anonymizer
      anonymizer, detection_threshold = anonymize_bag.init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters)


      # process left frame topic
      print("Processing left frame topic")
      begin = 0
      for left0, left1 in tqdm(zip_longest(iterator0_left, iterator1_left), total=left_frames0):
            
            if left1 is not None:
                  _, msg0, _ = left0
                  _, msg1, _ = left1

                  assert msg0.header.stamp.to_sec() == msg1.header.stamp.to_sec()

                  if begin == 0:
                        begin = msg0.header.stamp.to_sec()

                  if in_interval(begin, msg0.header.stamp.to_sec(), intervals):
                        bag_out.write(left_topic, msg0, msg0.header.stamp)

                  elif ifmask(begin, msg0.header.stamp.to_sec()):
                        image = convert_ros_img_to_img(msg0, 'bgr8', True)
                        left_mask = ['0,430,430,768','300,650,1024,768']
                        anonymized_image, _ = anonymizer.anonymize_image(image, detection_threshold, left_mask)
                        ros_image = convert_img_to_ros_img(anonymized_image, 'bgr8', msg0.header, True)
                        bag_out.write(left_topic, ros_image, ros_image.header.stamp)
                  else:
                        bag_out.write(left_topic, msg1, msg1.header.stamp)

            else:
                  _, msg0, _ = left0
                  image = convert_ros_img_to_img(msg0, 'bgr8', True)
                  anonymized_image, _ = anonymizer.anonymize_image(image, detection_threshold)
                  ros_image = convert_img_to_ros_img(anonymized_image, 'bgr8', msg0.header, True)
                  bag_out.write(left_topic, ros_image, ros_image.header.stamp)

      print("Processing right frame topic")
      # process right frame topic
      begin = 0
      for right0, right1 in tqdm(zip_longest(iterator0_right, iterator1_right), total=right_frames0):

            _, msg0, _ = right0
            _, msg1, _ = right1

            assert msg0.header.stamp.to_sec() == msg1.header.stamp.to_sec()

            if begin == 0:
                  begin = msg0.header.stamp.to_sec()
            
            if in_interval(begin, msg0.header.stamp.to_sec(), intervals):
                  bag_out.write(right_topic, msg0, msg0.header.stamp)
            elif ifmask(begin, msg0.header.stamp.to_sec()):
                  image = convert_ros_img_to_img(msg0, 'bgr8', True)
                  right_mask = ['0,390,400,768','0,620,680,768']
                  anonymized_image, _ = anonymizer.anonymize_image(image, detection_threshold, right_mask)
                  ros_image = convert_img_to_ros_img(anonymized_image, 'bgr8', msg0.header, True)
                  bag_out.write(right_topic, ros_image, ros_image.header.stamp)
            else:
                  bag_out.write(right_topic, msg1, msg1.header.stamp)
      
      print("close the bag for memory save")
      bag0.close()
      bag1.close()
      bag_out.close()
      print("Done")


if __name__ == "__main__":
      bag0 = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/highway00.bag')
      bag1 = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/highway00_anonymized.bag')
      bag_out = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/vehicle/highway00/highway00_refined00.bag', 'w')
      weights_path = '/home/jarvis/jw_ws/FusionPortable_utils/release/anonymizer/weights'
      intervals = [[50,62], [147, 162], [270,288], [293, 330], [343, 346], [353, 371], [510, 520], [626,628], [229, 285]]
      mask_interval = [0, 10]
      bag_merge(bag0, bag1, bag_out, intervals, weights_path)


# if __name__ == "__main__":
#       bag0 = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/handheld/room01/room01.bag')
#       bag1 = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/handheld/room01/room01_anonymized.bag')
#       weights_path = '/home/jarvis/jw_ws/FusionPortable_utils/release/anonymizer/weights'
#       bag_out = rosbag.Bag('/mnt/DATA_JW/FusionPortable_dataset_develop/sensor_data/handheld/room01/room01_refined00.bag', 'w')
#       complete_frame(bag0, bag1, bag_out, weights_path)





