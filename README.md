This project consists of two ROS2 nodes:

usb_cam: A node that captures video from a USB or laptop camera and publishes the images on a ROS2 topic.
<br>
image_conversion: A node that subscribes to the image topic published by usb_cam and converts the image to grayscale or color based on a service request. It publishes the converted image to a separate topic.
