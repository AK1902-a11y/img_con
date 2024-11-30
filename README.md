<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Image Conversion for USB Camera in ROS2</title>
</head>
<body>
    <h1>Image Conversion for USB Camera in ROS2</h1>

    <p>This project consists of two ROS2 nodes:</p>
    <ol>
        <li><strong>usb_cam</strong>: A node that captures video from a USB or laptop camera and publishes the images on a ROS2 topic.</li>
        <li><strong>image_conversion</strong>: A node that subscribes to the image topic published by <code>usb_cam</code> and converts the image to grayscale or color based on a service request. It publishes the converted image to a separate topic.</li>
    </ol>

    <h2>Table of Contents</h2>
    <ul>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#usage">Usage</a></li>
        <li><a href="#configuration">Configuration</a></li>
        <li><a href="#launch-files">Launch Files</a></li>
        <li><a href="#parameters">Parameters</a></li>
        <li><a href="#service-api">Service API</a></li>
        <li><a href="#example">Example</a></li>
    </ul>

    <h2 id="installation">Installation</h2>
    <h3>Prerequisites</h3>
    <ul>
        <li>ROS2 (Foxy, Galactic, or Humble recommended)</li>
        <li><code>usb_cam</code> package for ROS2 (used to stream images from the camera)</li>
    </ul>

    <h3>Install <code>usb_cam</code> package</h3>
    <p>First, install the <code>usb_cam</code> package to allow your system to access the USB camera:</p>
    <pre>
        sudo apt update
        sudo apt install ros-<ros2-distro>-usb-cam
    </pre>
    <p>Make sure to replace <code>&lt;ros2-distro&gt;</code> with your ROS2 version (e.g., <code>foxy</code>, <code>galactic</code>, <code>humble</code>).</p>

    <h3>Create ROS2 Package</h3>
    <ol>
        <li>Create a new ROS2 package called <code>image_conversion</code>:</li>
        <pre>ros2 pkg create --build-type ament_python image_conversion</pre>
        <li>Navigate to the <code>image_conversion</code> package:</li>
        <pre>cd ~/ros2_ws/src/image_conversion</pre>
        <li>Create the necessary files for your package:
            <ul>
                <li><code>image_conversion.py</code> (main Python node for image conversion)</li>
                <li><code>CMakeLists.txt</code> and <code>package.xml</code> (standard ROS2 package files)</li>
                <li><code>launch</code> folder to include launch files</li>
            </ul>
        </li>
        <li>Install required dependencies for the package (if necessary):</li>
        <pre>sudo apt install python3-opencv</pre>
        <li>Build the package:</li>
        <pre>cd ~/ros2_ws
        colcon build
        </pre>
    </ol>

    <h2 id="usage">Usage</h2>

    <h3>1. Launching the Nodes</h3>
    <p>Create a launch file to launch both the <code>usb_cam</code> node and the <code>image_conversion</code> node:</p>
    <pre>ros2 launch image_conversion image_conversion_launch.py</pre>

    <h3>2. Starting the Camera and Conversion Node</h3>
    <p>Once the launch file is executed, the USB camera will start publishing images on the ROS2 topic (e.g., <code>/usb_cam/image_raw</code>), and the <code>image_conversion</code> node will be ready to subscribe to this topic and convert images as per the mode selected.</p>

    <h2 id="configuration">Configuration</h2>

    <h3>1. Parameters</h3>
    <p>The ROS2 package allows changing the following parameters through the launch file:</p>
    <ul>
        <li><strong>input_camera_topic</strong>: The topic from which the image is being received (e.g., <code>/usb_cam/image_raw</code>).</li>
        <li><strong>output_image_topic</strong>: The topic where the converted image will be published (e.g., <code>/converted_image</code>).</li>
    </ul>

    <p>Example configuration in <code>image_conversion_launch.py</code>:</p>
    <pre>
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam',
            output='screen',
            parameters=[{'video_device': '/dev/video0'}],
            remappings=[('/image_raw', '/usb_cam/image_raw')]
        ),
        Node(
            package='image_conversion',
            executable='image_conversion',
            name='image_conversion',
            output='screen',
            parameters=[{'input_camera_topic': '/usb_cam/image_raw', 'output_image_topic': '/converted_image'}]
        ),
    ])
    </pre>

    <h3>2. ROS2 Service for Mode Change</h3>
    <p>The <code>image_conversion</code> node exposes a service to change the mode of image conversion:</p>
    <ul>
        <li><strong>Mode 1</strong>: Converts the image to grayscale.</li>
        <li><strong>Mode 2</strong>: Publishes the image without conversion (color mode).</li>
    </ul>
    <p>Service Type: <code>std_srvs/SetBool</code></p>
    <p>Input: <strong>True</strong> (Grayscale) or <strong>False</strong> (Color)</p>

    <h2 id="service-api">Service API</h2>

    <h3>Mode Change Service</h3>
    <p>To change the image conversion mode, you can call the ROS2 service from any other node or command line:</p>
    <pre>ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"  # Grayscale</pre>
    <pre>ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: false}"  # Color</pre>

    <h2 id="example">Example</h2>

    <h3>1. Launch both the camera and conversion nodes</h3>
    <pre>ros2 launch image_conversion image_conversion_launch.py</pre>

    <h3>2. Check the image topics</h3>
    <pre>ros2 topic list</pre>
    <p>This should show the image topics such as <code>/usb_cam/image_raw</code> and <code>/converted_image</code>.</p>

    <h3>3. Call the service to change the mode</h3>
    <pre>ros2 service call /image_conversion/set_mode std_srvs/srv/SetBool "{data: true}"  # Grayscale mode</pre>

    <h3>4. View the converted images in RViz or any image viewer</h3>
    <pre>ros2 topic echo /converted_image</pre>

    <h2>Conclusion</h2>
    <p>This project allows you to subscribe to an image topic from a USB camera, change the mode of the image (grayscale or color) via a service call, and publish the converted image to another ROS2 topic.</p>

    <h2>License</h2>
    <p>Include your licensing details if applicable. For example, MIT License or GPL-3.0 License.</p>
</body>
</html>
