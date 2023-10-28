# Author: khaclinh
import os
import torch
import numpy as np
from PIL import Image
import importlib
import cv2 

from yolox.exp import Exp as MyExp
from yolox.utils import postprocess
from yolox.data.data_augment import ValTransform


class Exp(MyExp):
    def __init__(self):
        super(Exp, self).__init__()
        self.depth = 1.0 # indicate size yolo model
        self.width = 1.0 #
        self.exp_name = os.path.split(os.path.realpath(__file__))[1].split(".")[0]

        self.data_dir = ''
        self.train_ann = ''
        self.val_ann = '' 
        self.test_ann = ''
        
        self.num_classes = 2
        self.data_num_workers = 32 # number of cpu for splitting batch
   
        self.input_size = (800, 800)   
        self.print_interval = 100
        self.eval_interval = 1
        self.test_size = (800, 800)
        self.enable_mixup = True
        self.mosaic_scale = (0.5, 1.5)
        self.max_epoch = 300 
        self.hsv_prob = 1.0

        self.degrees = 20.0
        self.translate = 0.2
        self.shear = 2.0
        # Turn off mosaic
        self.mosaic_prob = 1.0
        # Turn off Mixup
        self.mixup_prob = 1.0
        # Change SGD by ADAM
        

        self.basic_lr_per_img = 0.01 / 28.0
        self.no_aug_epochs = 15
        self.min_lr_ratio = 0.05
        self.ema = True

        self.nmsthre = 0.3


class Detector_pp4av():
    def __init__(self, exp: Exp, ckpt_path: str):
        self.exp = exp
        self.ckpt_path = ckpt_path


        


# get YOLOX experiment
exp = Exp()

# set inference parameters
test_size = (800, 800)
num_classes = 2
nmsthre = 0.3

GDPR_CLASSES = (
    "Face",
    "Plate"
)

# get YOLOX model
model = exp.get_model()
#model.cuda()
model.eval()

# get custom trained checkpoint
ckpt = torch.load(ckpt_file, map_location="")
model.load_state_dict(ckpt["model"])


def yolox_inference(img, model, prob_threshold, test_size): 
    bboxes = []
    bbclasses = []
    scores = []
    
    preproc = ValTransform(legacy = False)

    tensor_img, _ = preproc(img, None, test_size)
    tensor_img = torch.from_numpy(tensor_img).unsqueeze(0)
    tensor_img = tensor_img.float()
    #tensor_img = tensor_img.cuda()

    with torch.no_grad():
        outputs = model(tensor_img)
        outputs = postprocess(
                    outputs, num_classes, prob_threshold,
                    nmsthre, class_agnostic=True
                )

    if outputs[0] is None:
        return [], [], []
    
    outputs = outputs[0].cpu()
    bboxes = outputs[:, 0:4]

    bboxes /= min(test_size[0] / img.shape[0], test_size[1] / img.shape[1])
    bbclasses = outputs[:, 6]
    scores = outputs[:, 4] * outputs[:, 5]
    
    return bboxes, bbclasses, scores


def draw_yolox_predictions(img, bboxes, scores, bbclasses, prob_threshold, classes_dict):
    for i in range(len(bboxes)):
            box = bboxes[i]
            cls_id = int(bbclasses[i])
            score = scores[i]
            if score < prob_threshold:
                continue
            x0 = int(box[0])
            y0 = int(box[1])
            x1 = int(box[2])
            y1 = int(box[3])
            if cls_id == 0:

                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(img, '{}:{:.1f}%'.format(classes_dict[cls_id], score * 100), (x0, y0 - 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,255,0), thickness = 1)
            else:
                cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 0), 2)
                cv2.putText(img, '{}:{:.1f}%'.format(classes_dict[cls_id], score * 100), (x0, y0 - 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255,0,0), thickness = 1)
            
            
    return img


def pp4av_detect(img, prob_threshold=0.1):
    # Convert PIL image to CV2
    open_cv_image = np.array(img) 
    # Convert RGB to BGR 
    open_cv_image = open_cv_image[:, :, ::-1].copy() 

    bboxes, bbclasses, scores = yolox_inference(open_cv_image, model, prob_threshold, test_size)

    out = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2RGB)
    # Draw predictions
    out_image = draw_yolox_predictions(out, bboxes, scores, bbclasses, prob_threshold, GDPR_CLASSES)
    
    return Image.fromarray(out_image)
