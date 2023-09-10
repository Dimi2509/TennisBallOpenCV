import cv2 as cv
from objectDetection import Detector


def main():
    BallPicker = Detector()
    BallPicker.detect()
    

if __name__ == "__main__":
    main()

