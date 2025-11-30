import cv2
import numpy as np


if __name__ == "__main__":
    image = cv2.imread("image/test_image.jpg")

    lane_image = np.copy(image)

    # Convert to grayscale
    gray_scale_img = cv2.cvtColor(lane_image, cv2.COLOR_RGB2GRAY)

    # Apply gaussian filter to remove noise: 5x5 kernel, 0 standard deviation
    blur_img = cv2.GaussianBlur(gray_scale_img, (5, 5), 0)

    # Canny edge detection
    low_threshold = 50
    high_threshold = 150
    canny_img = cv2.Canny(blur_img, low_threshold, high_threshold)



    cv2.imshow("image", canny_img)
    cv2.waitKey(0)
