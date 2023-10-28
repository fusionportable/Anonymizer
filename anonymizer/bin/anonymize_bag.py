import sys
sys.path.append('../anonymizer')
import argparse
import rosbag
from tqdm import tqdm

from anonymizer.anonymization import Anonymizer
from anonymizer.detection import Detector, get_weights_path
from anonymizer.obfuscation import Obfuscator
from anonymizer.utils import load_bag_msg, convert_img_to_ros_img, convert_ros_img_to_img


def parse_args():
    parser = argparse.ArgumentParser(description='Anonymize faces and license plates of rosbags.')
    parser.add_argument('--input', required=True,
                        metavar='/path/to/input_folder',
                        help='Path to a folder that contains the rosbags that should be anonymized. '
                                'rosbags can be arbitrarily nested in subfolders and will still be found.')
    parser.add_argument('--output', required=True,
                        metavar='/path/to/output bag path/',
                        help='Path to the folder the anonymized rosbags should be written to. '
                                'Will mirror the folder structure of the input folder.')
    parser.add_argument('--weights', required=True,
                        metavar='/path/to/weights_foler',
                        help='Path to the folder where the weights are stored. If no weights with the '
                                'appropriate names are found they will be downloaded automatically.')
    parser.add_argument('--vehicle', required=False,
                        action='store_true',
                        metavar='/data/from/vehicle/platform',
                        help='If the data from vehicle, a special mask and ros topic name needs to be specified.')
    return parser.parse_args()


def init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters):
    kernel_size, sigma, box_kernel_size = obfuscation_parameters.split(',')
    obfuscator = Obfuscator(kernel_size=int(kernel_size), sigma=float(sigma), box_kernel_size=int(box_kernel_size))
    detectors = {
        'face': Detector(kind='face', weights_path=get_weights_path(weights_path, kind='face')),
        'plate': Detector(kind='plate', weights_path=get_weights_path(weights_path, kind='plate'))
    }
    detection_thresholds = {
        'face': face_threshold,
        'plate': plate_threshold
    }
    anonymizer = Anonymizer(obfuscator=obfuscator, detectors=detectors)
    return anonymizer, detection_thresholds


def vehicle(input_bag: rosbag.Bag, output_bag: rosbag.Bag, weights_path: str):
    # init the anonymizer
    face_threshold = 0.2
    plate_threshold = 0.2
    obfuscation_parameters = '21,2,9'
    anonymizer, detection_thresholds = init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters)
    
    # init the stereo topics
    topic_left = '/stereo/vehicle_frame_left/image_raw/compressed'
    topic_right = '/stereo/vehicle_frame_right/image_raw/compressed'
    mask_left = '300, 650, 1024, 768'
    mask_right = '0, 620, 680, 768'

    # init the image iterator
    image_iterator_left, total_frames = load_bag_msg(input_bag, topic_left)
    image_iterator_right, total_frames = load_bag_msg(input_bag, topic_right)


    # loop process
    for left, right in tqdm(zip(image_iterator_left, image_iterator_right), total=total_frames):
        _ , msg_left, _ = left
        _ , msg_right, _ = right
        image_left = convert_ros_img_to_img(msg_left, 'bgr8', True)
        image_right = convert_ros_img_to_img(msg_right, 'bgr8', True)
        anonymized_image_left, _ = anonymizer.anonymize_image(image_left, detection_thresholds, mask_left)
        anonymized_image_right, _ = anonymizer.anonymize_image(image_right, detection_thresholds, mask_right)
        ros_image_left = convert_img_to_ros_img(anonymized_image_left, 'bgr8', msg_left.header, True)
        ros_image_right = convert_img_to_ros_img(anonymized_image_right, 'bgr8', msg_right.header, True)
        output_bag.write(topic_left, ros_image_left, ros_image_left.header.stamp)
        output_bag.write(topic_right, ros_image_right, ros_image_right.header.stamp)
    output_bag.close()
    input_bag.close()



def main(input_bag: rosbag.Bag, output_bag: rosbag.Bag, weights_path: str):
    # init the anonymizer
    face_threshold = 0.2
    plate_threshold = 0.2
    obfuscation_parameters = '21,2,9'
    anonymizer, detection_thresholds = init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters)
    
    # init the stereo topics
    topic_left = '/stereo/frame_left/image_raw/compressed'
    topic_right = '/stereo/frame_right/image_raw/compressed'

    # init the image iterator
    image_iterator_left, total_frames = load_bag_msg(input_bag, topic_left)
    image_iterator_right, total_frames = load_bag_msg(input_bag, topic_right)


    # loop process
    for left, right in tqdm(zip(image_iterator_left, image_iterator_right), total=total_frames):
        _ , msg_left, _ = left
        _ , msg_right, _ = right
        image_left = convert_ros_img_to_img(msg_left, 'bgr8', True)
        image_right = convert_ros_img_to_img(msg_right, 'bgr8', True)
        anonymized_image_left, _ = anonymizer.anonymize_image(image_left, detection_thresholds)
        anonymized_image_right, _ = anonymizer.anonymize_image(image_right, detection_thresholds)
        ros_image_left = convert_img_to_ros_img(anonymized_image_left, 'bgr8', msg_left.header, True)
        ros_image_right = convert_img_to_ros_img(anonymized_image_right, 'bgr8', msg_right.header, True)
        output_bag.write(topic_left, ros_image_left, ros_image_left.header.stamp)
        output_bag.write(topic_right, ros_image_right, ros_image_right.header.stamp)
    output_bag.close()
    input_bag.close()

if __name__ == "__main__":
    args = parse_args()
    if args.vehicle:
        vehicle(rosbag.Bag(args.input), rosbag.Bag(args.output, 'w'), args.weights)
    else:
        main(rosbag.Bag(args.input), rosbag.Bag(args.output, 'w'), args.weights)