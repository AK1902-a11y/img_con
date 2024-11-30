<h1>Image conversion for USB Camera in ROS2</h1>
<br>
<h4>This project consists of two ROS2 nodes:</h4>
<br>
usb_cam: A node that captures video from a USB or laptop camera and publishes the images on a ROS2 topic.
<br>
image_conversion: A node that subscribes to the image topic published by usb_cam and converts the image to grayscale or color based on a service request. It publishes the converted image to a separate topic.
<br>
<h3>Setup</h3> <br>
ROS2 Humble <br>
usb_cam package
<br>
<h3>Usage</h3> 
<br>
Launching Node: <br><br>
<b><code>ros2 launch image_conversion image_conversion_launch.py</code></b>
<br><br>
Once the launch file is executed, the USB camera will start publishing images on the ROS2 topic (e.g./image_raw), and the image_conversion node will be ready to subscribe to this topic and convert images as per the mode selected.
<br>
<h3>Configuration</h3> <br>
1)Parameters: The ROS2 package allows changing the following parameters through the launch file: <br><br>

input_camera_topic: The topic from which the image is being received (e.g., /usb_cam/image_raw). <br>
output_image_topic: The topic where the converted image will be published (e.g., /converted_image). <br>

2)ROS2 Service for Mode Change: The image_conversion node exposes a service to change the mode of image conversion: <br><br>
Mode 1: Converts the image to grayscale.<br>
Mode 2: Publishes the image without conversion (color mode).
<br>
<h3>Service API</h3>
<br>

To change the image conversion mode, you can call the ROS2 service from any other node or command line (mentioned in service call text file in image conversion package): <br><br>
<b><code>ros2 service call /change_image_mode std_srvs/srv/SetBool "{data: true}"</code></b>  # To set to greyscale mode<br>
<b><code>ros2 service call /change_image_mode std_srvs/srv/SetBool "{data: false}"</code><b>  # Color <br>



