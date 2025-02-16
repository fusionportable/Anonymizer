import argparse
import sys
sys.path.append('../anonymizer')
import onnxruntime as ort
import cv2
from pathlib import Path
from tqdm import tqdm
from PIL import Image
import numpy as np

from anonymizer.anonymization import AnonymizerOnnx
from anonymizer.detection import Detector
from anonymizer.obfuscation import Obfuscator


def parse_args():
    parser = argparse.ArgumentParser(
        description='Anonymize faces and license plates in a series of images.')
    parser.add_argument('--input', required=True,
                        metavar='/path/to/input_folder',
                        help='Path to a folder that contains the images that should be anonymized. '
                             'Images can be arbitrarily nested in subfolders and will still be found.')
    parser.add_argument('--output', required=True,
                        metavar='/path/to/output_foler',
                        help='Path to the folder the anonymized images should be written to. '
                             'Will mirror the folder structure of the input folder.')
    parser.add_argument('--weights', required=True,
                        metavar='/path/to/weights_foler',
                        help='Path to the folder where the weights are stored. If no weights with the '
                             'appropriate names are found they will be downloaded automatically.')
    parser.add_argument('--image-extensions', required=False, default='jpg,png',
                        metavar='"jpg,png"',
                        help='Comma-separated list of file types that will be anonymized')
    parser.add_argument('--face-threshold', type=float, required=False, default=0.3,
                        metavar='0.3',
                        help='Detection confidence needed to anonymize a detected face. '
                             'Must be in [0.001, 1.0]')
    parser.add_argument('--plate-threshold', type=float, required=False, default=0.3,
                        metavar='0.3',
                        help='Detection confidence needed to anonymize a detected license plate. '
                             'Must be in [0.001, 1.0]')
    parser.add_argument('--write-detections', dest='write_detections', action='store_true')
    parser.add_argument('--no-write-detections', dest='write_detections', action='store_false')
    parser.set_defaults(write_detections=False)
    parser.add_argument('--obfuscation-kernel', required=False, default='21,2,9',
                        metavar='kernel_size,sigma,box_kernel_size',
                        help='This parameter is used to change the way the blurring is done. '
                             'For blurring a gaussian kernel is used. The default size of the kernel is 21 pixels '
                             'and the default value for the standard deviation of the distribution is 2. '
                             'Higher values of the first parameter lead to slower transitions while blurring and '
                             'larger values of the second parameter lead to sharper edges and less blurring. '
                             'To make the transition from blurred areas to the non-blurred image smoother another '
                             'kernel is used which has a default size of 9. Larger values lead to a smoother '
                             'transition. Both kernel sizes must be odd numbers.')
    args = parser.parse_args()

    print(f'input: {args.input}')
    print(f'image-output: {args.output}')
    print(f'weights: {args.weights}')
    print(f'image-extensions: {args.image_extensions}')
    print(f'face-threshold: {args.face_threshold}')
    print(f'plate-threshold: {args.plate_threshold}')
    print(f'write-detections: {args.write_detections}')
    print(f'obfuscation-kernel: {args.obfuscation_kernel}')
    print()

    return args


def main(providers, input_path, image_output_path, weights_path, image_extensions, face_threshold, plate_threshold,
         write_json, obfuscation_parameters):
      
      kernel_size, sigma, box_kernel_size = obfuscation_parameters.split(',')
      obfuscator = Obfuscator(kernel_size=int(kernel_size), sigma=float(sigma), box_kernel_size=int(box_kernel_size))
      
      detector = {
            'face': Detector(kind='face', weights_path=weights_path, providers = providers),
            'plate': Detector(kind='plate', weights_path=weights_path, providers = providers),
      }
      detection_thresholds = {
        'face': face_threshold,
        'plate': plate_threshold
      }

      anonymizer = AnonymizerOnnx(detectors=detector, obfuscator=obfuscator)
      anonymizer.anonymize_images(input_path=input_path, output_path=image_output_path,
                                detection_thresholds=detection_thresholds, file_types=image_extensions.split(','),
                                write_json=write_json)

      # if not Path(output_path).is_dir():
      #       # create the output directory
      #       Path(output_path).mkdir(parents=True, exist_ok=True)

      # images = list(Path(input_path).glob('*.png'))

      # for image in tqdm(images):
      #       name = str(image).split('/')[-1]
      #       image = load_np_image(image)
      #       detected_boxes = detect_image(image, detector, detection_thresholds)
      #       anonymized_image = obfuscator.obfuscate(image, detected_boxes)
      #       # vis_image = draw_bboxes(anonymized_image, detected_boxes)
      #       save_np_image(anonymized_image, Path(output_path, name))


# def main():
#       providers = [("CUDAExecutionProvider", {"device_id": '0'}), 'CPUExecutionProvider']
#       args = parse_args()
#       main(providers, input_path=args.input, image_output_path=args.image_output, weights_path=args.weights,
#          image_extensions=args.image_extensions,
#          face_threshold=args.face_threshold, plate_threshold=args.plate_threshold,
#          write_json=args.write_detections, obfuscation_parameters=args.obfuscation_kernel)

      # input_path = "/workspace/anonymizer/data/handheld/room00/frame_left_raw"
      # output_path = "/workspace/anonymizer/data/handheld/room00/frame_left_anonymization"      
      # weights_path = "/workspace/anonymizer/weights"
      # face_threshold = 0.3
      # plate_threshold = 0.3
      # test_anonymizer(providers, input_path, output_path, weights_path, face_threshold, plate_threshold)

if __name__ == "__main__":
      providers = [("CUDAExecutionProvider", {"device_id": '0'}), 'CPUExecutionProvider']
      args = parse_args()
      main(providers, input_path=args.input, image_output_path=args.output, weights_path=args.weights,
         image_extensions=args.image_extensions,
         face_threshold=args.face_threshold, plate_threshold=args.plate_threshold,
         write_json=args.write_detections, obfuscation_parameters=args.obfuscation_kernel)
