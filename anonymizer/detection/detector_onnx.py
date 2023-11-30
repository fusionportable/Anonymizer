import numpy as np
import onnxruntime as ort

from anonymizer.utils import Box
from pathlib import Path


KINDS = {
    'face': 'face.onnx',
    'plate': 'plate.onnx'
}

class Detector:
    def __init__(self, kind, weights_path, providers):
        self.kind = kind
        self.providers = providers
        self.weights = self.get_weights(kind, weights_path)
        self.session = ort.InferenceSession(self.weights, providers=self.providers)

    def get_weights(self, kind, weights_path):
        assert kind in KINDS.keys(), f'Invalid kind "{kind}"'
        return str(Path(weights_path) / KINDS[kind])

    def _convert_boxes(self, num_boxes, scores, boxes, image_height, image_width, detection_threshold):
        assert detection_threshold >= 0.001, 'Threshold can not be too close to "0".'

        result_boxes = []
        for i in range(int(num_boxes)):
            score = float(scores[i])
            if score < detection_threshold:
                continue
            y_min, x_min, y_max, x_max = map(float, boxes[i].tolist())
            box = Box(y_min=y_min * image_height, x_min=x_min * image_width,
                      y_max=y_max * image_height, x_max=x_max * image_width,
                      score=score, kind=self.kind)
            result_boxes.append(box)
        return result_boxes

    def detect(self, image, detection_threshold):

        image_height, image_width, channels = image.shape
        assert channels == 3, f'Invalid number of channels: {channels}. ' \
                              f'Only images with three color channels are supported.'

        np_images = np.array([image])
        num_boxes, scores, boxes = self.session.run(None, {'image_tensor:0': np_images})
        
        # print(num_boxes)
        converted_boxes = self._convert_boxes(num_boxes=num_boxes[0], scores=scores[0], boxes=boxes[0],
                                              image_height=image_height, image_width=image_width,
                                              detection_threshold=detection_threshold)
        return converted_boxes
