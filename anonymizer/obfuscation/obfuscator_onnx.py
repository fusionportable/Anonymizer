import math

import numpy as np
import scipy.stats as st
import cv2

class Obfuscator:
    """ This class is used to blur box regions within an image with gaussian blurring. """
    def __init__(self, kernel_size=21, sigma=2, channels=3, box_kernel_size=9, smooth_boxes=True):
        """
        :param kernel_size: Size of the blurring kernel.
        :param sigma: standard deviation of the blurring kernel. Higher values lead to sharper edges, less blurring.
        :param channels: Number of image channels this blurrer will be used for. This is fixed as blurring kernels will
            be created for each channel only once.
        :param box_kernel_size: This parameter is only used when smooth_boxes is True. In this case, a smoothing
            operation is applied on the bounding box mask to create smooth transitions from blurred to normal image at
            the bounding box borders.
        :param smooth_boxes: Flag defining if bounding box masks borders should be smoothed.
        """
        # Kernel must be uneven because of a simplified padding scheme
        assert kernel_size % 2 == 1

        self.kernel_size = kernel_size
        self.box_kernel_size = box_kernel_size
        self.sigma = sigma
        self.channels = channels
        self.smooth_boxes = smooth_boxes

        # create internal kernels (3D kernels with the channels in the last dimension)
        self.kernel = self._gaussian_kernel(kernel_size=self.kernel_size, sigma=self.sigma)  # kernel for blurring
        # self.mean_kernel = self._bilinear_filter(filter_size=(box_kernel_size, box_kernel_size))  # kernel for smoothing
        self.mean_kernel = self._mean_kernel(kernel_size=box_kernel_size)

    def _mean_kernel(self, kernel_size=3):
        """ Returns a 2D mean kernel array.

        :param kernel_size: Size of the kernel, the resulting array will be kernel_size x kernel_size
        :return: 2D numpy array containing a mean kernel.
        """

        kernel = np.ones((kernel_size, kernel_size), np.float64) / (kernel_size**2)

        return kernel


    def _bilinear_filter(self, filter_size=(4, 4)):
        """
        Make a 2D bilinear kernel suitable for upsampling of the given (h, w) size.
        Also allows asymmetric kernels.

        :param filter_size: Tuple defining the filter size in width and height.

        :return: 2D numpy array containing bilinear weights.
        """
        assert isinstance(filter_size, (list, tuple)) and len(filter_size) == 2

        factor = [(size + 1) // 2 for size in filter_size]
        # define first center dimension
        if filter_size[0] % 2 == 1:
            center_x = factor[0] - 1
        else:
            center_x = factor[0] - 0.5
        # define second center dimension
        if filter_size[1] % 2 == 1:
            center_y = factor[1] - 1
        else:
            center_y = factor[1] - 0.5

        og = np.ogrid[:filter_size[0], :filter_size[1]]
        kernel = (1 - abs(og[0] - center_x) / float(factor[0])) * (1 - abs(og[1] - center_y) / float(factor[1]))

        return np.array(kernel, np.float64)


    def _gaussian_kernel(self, kernel_size=30, sigma=5):
        """ Returns a 2D Gaussian kernel array.

        :param kernel_size: Size of the kernel, the resulting array will be kernel_size x kernel_size
        :param sigma: Standard deviation of the gaussian kernel.
        :return: 2D numpy array containing a gaussian kernel.
        """

        interval = (2 * sigma + 1.) / kernel_size
        x = np.linspace(-sigma - interval / 2., sigma + interval / 2., kernel_size + 1)
        kern1d = np.diff(st.norm.cdf(x))
        kernel_raw = np.sqrt(np.outer(kern1d, kern1d))
        kernel = kernel_raw / kernel_raw.sum()

        return kernel
    

    def run(self, image, mask):
        anonymized_image = image.copy()

        if self.smooth_boxes:
            smoothed_mask = cv2.filter2D(mask, ddepth=-1, kernel = self.mean_kernel)
            # print(smoothed_mask)
        else:
            smoothed_mask = mask
        
        blurred_image = cv2.filter2D(image, ddepth = -1, kernel = self.kernel)

        smoothed_mask_3d = np.stack([smoothed_mask, smoothed_mask, smoothed_mask], axis=-1)
        anonymized_image = image * (1-smoothed_mask_3d) + blurred_image * smoothed_mask_3d

        # anonymized_image[:,:,0] = image[:,:,0] * (1-smoothed_mask) + blurred_image[:,:,0]* smoothed_mask
        # anonymized_image[:,:,1] = image[:,:,1] * (1-smoothed_mask) + blurred_image[:,:,1]* smoothed_mask
        # anonymized_image[:,:,2] = image[:,:,2] * (1-smoothed_mask) + blurred_image[:,:,2]* smoothed_mask

        self.image = image
        self.mask = mask
        self.anonymized_image = anonymized_image

        return anonymized_image


    def _get_all_masks(self, bboxes, image):
        
        mask = np.zeros(shape=(image.shape[0], image.shape[1]))
        # insert box masks into array
        for box in bboxes:
            mask[box[1]:box[3], box[0]:box[2]] = 1

        return mask
    

    def _get_box_mask(self, bboxes):
        """
        :return bbox_array: (N, 4) array of bounding boxes with quantized values
        """

        box_array = []
        for box in bboxes:
            x_min = int(math.floor(box.x_min))
            y_min = int(math.floor(box.y_min))
            x_max = int(math.ceil(box.x_max))
            y_max = int(math.ceil(box.y_max))
            box_array.append(np.array([x_min, y_min, x_max, y_max]))
        
        return np.stack(box_array, axis=0)
        

    def _obfuscate_image(self, image, bboxes):
        """
        bbox_mask = (H, W) with binary values on mask area

        :return: The anonymized image.
        """

        # quantized bboxes
        bbox_array = self._get_box_mask(bboxes)
        # transfer to binary mask
        bbox_mask = self._get_all_masks(bboxes=bbox_array, image=image)
        # obfuscate the boxes
        anonymized_image = self.run(image, bbox_mask)

        return anonymized_image

    def obfuscate(self, image, boxes):
        """
        Anonymize all bounding boxes in a given image.
        :param image: The image as np.ndarray with shape==(height, width, channels).
        :param boxes: A list of boxes.
        :return: The anonymized image.
        """
        # if no boxes, return the original image
        if len(boxes) == 0:
            return np.copy(image)

        anonymized_image = self._obfuscate_image(image, boxes)

        return anonymized_image
