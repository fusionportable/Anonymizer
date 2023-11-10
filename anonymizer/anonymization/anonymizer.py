import json
from pathlib import Path

import numpy as np
from PIL import Image
from tqdm import tqdm


def load_np_image(image_path):
    image = Image.open(image_path).convert('RGB')
    np_image = np.array(image)
    return np_image


def save_np_image(image, image_path):
    pil_image = Image.fromarray((image).astype(np.uint8), mode='RGB')
    pil_image.save(image_path)


def save_detections(detections, detections_path):
    json_output = []
    for box in detections:
        json_output.append({
            'y_min': box.y_min,
            'x_min': box.x_min,
            'y_max': box.y_max,
            'x_max': box.x_max,
            'score': box.score,
            'kind': box.kind
        })
    with open(detections_path, 'w') as output_file:
        json.dump(json_output, output_file, indent=2)


def inmask(box, masks):
    if isinstance(masks, str):
        masks = [masks]

    for mask in masks:
        x_min, y_min, x_max, y_max = box.x_min, box.y_min, box.x_max, box.y_max
        x_min_mask, y_min_mask, x_max_mask, y_max_mask = mask.split(',')
        x_min_mask = int(x_min_mask)
        y_min_mask = int(y_min_mask)
        x_max_mask = int(x_max_mask)
        y_max_mask = int(y_max_mask)
        lower_bound = x_min >= x_min_mask and y_min >= y_min_mask
        higher_bound = x_max <= x_max_mask and y_max <= y_max_mask
        if (higher_bound and lower_bound):
            return True
        
    return False


class Anonymizer:
    def __init__(self, detectors, obfuscator):
        self.detectors = detectors
        self.obfuscator = obfuscator

    def anonymize_image(self, image, detection_thresholds, mask = None):
        assert set(self.detectors.keys()) == set(detection_thresholds.keys()),\
            'Detector names must match detection threshold names'
        detected_boxes = []
        for kind, detector in self.detectors.items():
            new_boxes = detector.detect(image, detection_threshold=detection_thresholds[kind])
            if kind == "plate":
                if mask is not None:
                    # x_min, y_min, x_max, y_max = mask.split(',')
                    # x_min = int(x_min)
                    # y_min = int(y_min)
                    # x_max = int(x_max)
                    # y_max = int(y_max)
                    masked_boxes = []
                    for new_box in new_boxes:
                        if not inmask(new_box, mask):
                            masked_boxes.append(new_box)
                        # x_min_new, y_min_new, x_max_new, y_max_new = new_box.x_min, new_box.y_min, new_box.x_max, new_box.y_max
                        # lower_bound = x_min_new >= x_min and y_min_new >= y_min
                        # higher_bound = x_max_new <= x_max and y_max_new <= y_max
                        # if not (lower_bound and higher_bound):
                        #     masked_boxes.append(new_box)
                    new_boxes = masked_boxes
            detected_boxes.extend(new_boxes)
        return self.obfuscator.obfuscate(image, detected_boxes), detected_boxes

    def anonymize_images(self, input_path, output_path, detection_thresholds, file_types, write_json, mask):
        print(f'Anonymizing images in {input_path} and saving the anonymized images to {output_path}...')

        Path(output_path).mkdir(exist_ok=True)
        assert Path(output_path).is_dir(), 'Output path must be a directory'

        files = []
        for file_type in file_types:
            print(list(Path(input_path).glob(f'**/*.{file_type}')))
            files.extend(list(Path(input_path).glob(f'**/*.{file_type}')))
        
        print(len(files))
        print(files)

        for input_image_path in tqdm(files):
            # Create output directory
            relative_path = input_image_path.relative_to(input_path)
            (Path(output_path) / relative_path.parent).mkdir(exist_ok=True, parents=True)
            output_image_path = Path(output_path) / relative_path
            output_detections_path = (Path(output_path) / relative_path).with_suffix('.json')

            # Anonymize image
            image = load_np_image(str(input_image_path))
            anonymized_image, detections = self.anonymize_image(image=image, detection_thresholds=detection_thresholds, mask=mask)
            save_np_image(image=anonymized_image, image_path=str(output_image_path))
            if write_json:
                save_detections(detections=detections, detections_path=str(output_detections_path))
