import yaml
from pathlib import Path
import os
import numpy as np

def load_matrix_distortion(Camera_Name: str):
    path = os.path.dirname(__file__)
    with open(path + "/camera_calibration" + "/calibration_matrices.yaml") as file:
        data = yaml.safe_load(file)
    
    print(data[Camera_Name])
    camera = data[Camera_Name]
    Kmatrix = camera["camera_matrix"]["data"]
    dist_coeffs = camera["distortion_coefficients"]["data"]
    print(f"Kmatix: {Kmatrix}")
    Kmatrix = np.asarray(Kmatrix).reshape((3,3))
    dist_coeffs = np.asarray(dist_coeffs)
    return Kmatrix, dist_coeffs

# load_matrix_distortion("usb-Framework_Laptop_Webcam_Module__2nd_Gen__FRANJBCHA1537100EB-video-index0")
    # ros2 run camera_calibration cameracalibrator --size 4x4 --square 0.0347 image:=/image_raw camera:=/webcam


def load_width_height(Camera_Name: str):
    path = os.path.dirname(__file__)
    with open(path + "/camera_calibration" + "/calibration_matrices.yaml") as file:
        data = yaml.safe_load(file)
    
    print(data[Camera_Name])
    camera = data[Camera_Name]
    width = camera["image_width"]
    height = camera["image_height"]
    return width, height
