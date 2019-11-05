#!/usr/bin/env python
from ada_demos.srv import DetectAcquisition, DetectAcquisitionResponse
from google_drive_downloader import GoogleDriveDownloader as gdd
import numpy as np
from os import path
import rospy
from sensor_msgs.msg import CompressedImage
import torch
from torchvision import models, transforms


MODEL_PATH = 'detector_ckpt.pth'


def detector():
    rospy.init_node('acquisition_detector')

    model = InferenceModel()
    s = rospy.Service('acquisition_detector', DetectAcquisition, model.handle_detection)

    rospy.spin()


class InferenceModel(object):
    def __init__(self):
        # Download the model file
        if not path.exists(MODEL_PATH):
            download_model_file()

        # Initialize and load model
        self.model = models.squeezenet1_0(pretrained=False)
        self.model.classifier[1] = torch.nn.Conv2d(512, 2, kernel_size=(1, 1), stride=(1, 1))
        self.model.load_state_dict(torch.load(MODEL_PATH))

        # Transforms
        self.color_normalizer = transforms.Normalize(mean=[70.7759, 84.4663, 91.1007],
                                                     std=[71.4193, 57.9395, 61.5698])

        #self.device = get_device(args)
        #self.model = model.to(self.device)
        self.model.eval()

    def handle_detection(self, req):
        msg = rospy.wait_for_message('/camera/color/image_raw/compressed', CompressedImage)
        #msg = rospy.wait_for_message('/camera/color/image_raw/', msg_type)

        # Tensor transform
        np_arr = np.fromstring(msg.data, np.uint8)
        img = torch.tensor(np_arr)
        img = self.color_normalizer(img)

        #inputs = sample['image'].to(device)
        with torch.set_grad_enabled(False):
            outputs = self.model(img)
            _, preds = torch.max(outputs, 1)

        return DetectAcquisitionResponse(bool(preds))


def download_model_file():
    gdd.download_file_from_google_drive(file_id='13OHm_qjaZTYrqsG6NG2IMlStBNdo93g4',
                                        dest_path=MODEL_PATH,
                                        unzip=False)


if __name__ == "__main__":
    detector()
