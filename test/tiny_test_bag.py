"""
      A test script for this project
"""
import sys
sys.path.append('../anonymizer')

import rosbag
from tqdm import tqdm
import cv2
import os
from pathlib import Path


from anonymizer.anonymization import *
from anonymizer.detection import Detector, download_weights, get_weights_path
from anonymizer.obfuscation import Obfuscator


def extract_tiny_bag(bag_in: rosbag.Bag, bag_out: rosbag.Bag, topic: str, start_frame: int, end_frame: int):
      """
      Extract a list of topics from a ROS bag to a new bag file.
      
      Args:
            bag: 
                  rosbag.Bag: the bag to extract from
            topic: 
                  str: the topics to extract
      
      Returns:
            None
      """

      index = 1
      total_frames = bag_in.get_message_count(topic_filters=topic)
      print(f"{total_frames} in {bag_in.filename}")
      print(f"Extracting {topic} from Frame ID: {start_frame} to {end_frame}")
      
      msgs = bag_in.read_messages(topics=topic)
      
      for _, msg, t in tqdm(msgs, total=total_frames):
            if index >= start_frame and index <= end_frame:
                  bag_out.write(topic, msg, t)
            index += 1

      bag_in.close()
      bag_out.close()
      print(f"Extracted {topic} from Frame ID: {start_frame} to {end_frame}")
      
      return bag_out.filename


def draw_bbox(image, bbox):
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)


def test_vehicle_mask(input_path, output_path, weights_path):
    obfuscation_parameters = '21,2,9'
    kernel_size, sigma, box_kernel_size = obfuscation_parameters.split(',')
    obfuscator = Obfuscator(kernel_size=int(kernel_size), sigma=float(sigma), box_kernel_size=int(box_kernel_size))
    detectors = {
        'face': Detector(kind='face', weights_path=get_weights_path(weights_path, kind='face')),
        'plate': Detector(kind='plate', weights_path=get_weights_path(weights_path, kind='plate'))
    }
    detection_thresholds = {
        'face': 0.3,
        'plate': 0.2
    }
    anonymizer = Anonymizer(obfuscator=obfuscator, detectors=detectors)
    anonymizer.anonymize_images(input_path, output_path, detection_thresholds, file_types='jpg,png'.split(','), write_json=True, mask='300,650,1024,768')


def parser():
      import argparse
      parser = argparse.ArgumentParser(description='Extract a topic from a ROS bag to a new bag file.')
      parser.add_argument('--input', required=True,
                        metavar='/path/to/input.bag',
                        help='Path to the input bag.')
      parser.add_argument('--output', required=True,
                        metavar='/path/to/output.bag',
                        help='Path to the output bag.')
      parser.add_argument('--topic', required=True,
                        metavar='/topic/to/extract',
                        help='Topic to extract from the bag.')
      parser.add_argument('--start_frame', required=True,
                        metavar='start_frame',
                        help='Start frame ID to extract from the bag.')
      parser.add_argument('--end_frame', required=True,
                        metavar='end_frame',
                        help='End frame ID to extract from the bag.')
      return parser.parse_args()


if __name__ == "__main__":
      # args = parser()
      # mask = '300,650,1024,768'
      # x_min, y_min, x_max, y_max = mask.split(',')
      # print(int(x_min), int(y_min), int(x_max), int(y_max))


      root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
      weights_path = os.path.join(root_dir, "weights/")
      print(weights_path)
      input_path = os.path.join(root_dir, "test/test_data/test_logo")
      print(input_path)
      output_path = os.path.join(root_dir, "test/test_data/test_logo_masked")
      print(output_path)
      Path(output_path).mkdir(exist_ok=True)
      assert Path(output_path).is_dir(), 'Output path must be a directory'
      test_vehicle_mask(input_path, output_path, weights_path)
      
      
      # files = []
      # files.extend(list(Path(input_path).glob(f'**/*.png')))
      # print(files)
      # for input_image_path in files:
      #       relative_path = input_image_path.relative_to(input_path)
      #       (Path(output_path) / relative_path.parent).mkdir(exist_ok=True, parents=True)
      #       output_image_path = Path(output_path) / relative_path
      #       # output_detections_path = (Path(output_path) / relative_path).with_suffix('.json')
      #       image = load_np_image(str(input_image_path))
      #       image_out, detections = test_vehicle_mask(image, "weights/")
      #       save_np_image(image_out, str(output_image_path))

      
      # image_out, detections = test_vehicle_mask(image, weights_path)
      # bag_in = rosbag.Bag(args.input)
      # bag_out = rosbag.Bag(args.output, 'w')
      # extract_tiny_bag(bag_in, bag_out, args.topic, int(args.start_frame), int(args.end_frame))
      # root_dir = os.path.dirname(os.path.abspath(__file__))
      # image_dir = os.path.join(root_dir, "test_data/right.png")
      # print(image_dir)
      # image = cv2.imread(image_dir)
      # print(image.shape)
      # # bbox for left: [300, 650, 1024, 768]
      # # bbox for right: [0, 620, 680, 768]
      # bbox = [0, 620, 680, 768]
      # draw_bbox(image, bbox)
      # cv2.imwrite(image_dir, image)

      