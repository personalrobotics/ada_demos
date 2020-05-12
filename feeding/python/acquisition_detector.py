#!/usr/bin/env python
from ada_demos.srv import DetectAcquisition, DetectAcquisitionResponse
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from os import path
import rospy
from sensor_msgs.msg import CompressedImage, Image
import sys
import torch
import torchvision
from torchvision import models, transforms


REL_MODEL_PATH = '../bash_scripts/checkpoint/squeezenet.pth'


def detector():
    rospy.init_node('acquisition_detector')

    model = InferenceModel()
    s = rospy.Service('acquisition_detector', DetectAcquisition, model.handle_detection)

    rospy.spin()


class InferenceModel(object):
    """
    Model class for performing inference from a pre-trained model.
    """
    def __init__(self):
        print('Torch version {}'.format(torch.__version__))
        print('Torchvision version {}'.format(torchvision.__version__))
        print('Python version {}'.format(sys.version_info))

        # Download the model file
        model_path = path.join(path.dirname(__file__), REL_MODEL_PATH)
        if not path.exists(model_path):
            OSError('Acquisition detector checkpoint not found. Run ada_demos/feeding/bash_scripts/download_detector_checkpoint.sh.')

        # Initialize and load model
        self.model = models.squeezenet1_0(pretrained=False)
        self.model.classifier[1] = torch.nn.Conv2d(512, 2, kernel_size=(1, 1), stride=(1, 1))
        self.model.num_classes = 2
        self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))

        # Transforms
        self.color_normalizer = transforms.Normalize(mean=[70.7759, 84.4663, 91.1007],
                                                     std=[71.4193, 57.9395, 61.5698])

        self.model.eval()

        self.bridge = CvBridge()

    def handle_detection(self, req):
        """
        Handle a detection service request.
        :param req:                         Request message.
        :return DetectAcquisitionResponse:  Detection response, containing boolean value success.
        """
        msg = rospy.wait_for_message('/camera/color/image_raw/', Image, timeout=1.0)

        # Tensor transform
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as s:
            print(s)
        img = np.asarray(cv_image).astype(np.float32)
        img = img.transpose((2, 0, 1))  # HWC --> CHW
        img = torch.tensor(img)
        img = self.color_normalizer(img)
        img.unsqueeze_(0)  # batch size 1

        with torch.set_grad_enabled(False):
            outputs = self.model(img)
            _, preds = torch.max(outputs, 1)

        return DetectAcquisitionResponse(bool(preds))


if __name__ == "__main__":
    detector()
