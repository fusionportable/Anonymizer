import sys
import argparse
import glob
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append(f"{parent_dir}/anonymizer")
sys.path.append(parent_dir)
# print(sys.path)
from tqdm import tqdm
from PIL import Image

from anonymizer.anonymization import AnonymizerOnnx
from anonymizer.detection import Detector
from anonymizer.obfuscation import Obfuscator

# import utils

def parse_args():
    parser = argparse.ArgumentParser(description='Tiny test of Anonymizer faces and license plates.')
    parser.add_argument('--input', required=True,
                        metavar='/path/to/input_folder',
                        help='Path to a folder that contains the images that should be anonymized.')
    parser.add_argument('--output', required=True,
                        metavar='/path/to/output_foler',
                        help='Path to the folder the anonymized images should be written to. '
                                'Will mirror the folder structure of the input folder.')
    parser.add_argument('--weights', required=True,
                        metavar='/path/to/weights_foler',
                        help='Path to the folder where the weights are stored. If no weights with the '
                                'appropriate names are found they will be downloaded automatically.')
    parser.add_argument('--face-threshold', type=float, required=False, default=0.3,
                        metavar='0.3',
                        help='Detection confidence needed to anonymize a detected face. '
                                'Must be in [0.001, 1.0]')
    parser.add_argument('--plate-threshold', type=float, required=False, default=0.3,
                        metavar='0.3',
                        help='Detection confidence needed to anonymize a detected license plate. '
                                'Must be in [0.001, 1.0]')
    parser.add_argument('--ext', required=False, default='jpg',
                        help='Image extension')
    return parser.parse_args()

class ImagesDataset:
    def __init__(self, folder_path, _ext = '.png'):
        self.folder_path = folder_path
        self.image_paths = []
        self.image_paths = glob.glob(f"{folder_path}/*{_ext}")
        self.image_paths.sort()
        self.total_frames = len(self.image_paths)

    def __len__(self):
        return self.total_frames

    def __iter__(self):
        for image_path in self.image_paths:
            yield image_path


def init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters, providers):
    kernel_size, sigma, box_kernel_size = obfuscation_parameters.split(',')
    obfuscator = Obfuscator(kernel_size=int(kernel_size), sigma=float(sigma), box_kernel_size=int(box_kernel_size))
    detectors = {
        'face': Detector(kind='face', weights_path=weights_path, providers=providers),
        'plate': Detector(kind='plate', weights_path=weights_path, providers=providers)
    }
    detection_thresholds = {
        'face': face_threshold,
        'plate': plate_threshold
    }
    anonymizer = AnonymizerOnnx(obfuscator=obfuscator, detectors=detectors)
    return anonymizer, detection_thresholds


def main(args, providers):
    # init the anonymizer
    weights_path = args.weights
    face_threshold = args.face_threshold
    plate_threshold = args.plate_threshold
    obfuscation_parameters = '21,2,9'
    anonymizer, detection_thresholds = init_anonymizer(weights_path, face_threshold, plate_threshold, obfuscation_parameters, providers)
    
    # run the anonymizer
    anonymizer.anonymize_images(args.input, args.output, detection_thresholds, [args.ext], False)

if __name__ == "__main__":
    providers = [("CUDAExecutionProvider", {"device_id": '0'}), 'CPUExecutionProvider']
    args = parse_args()
    main(args, providers)




