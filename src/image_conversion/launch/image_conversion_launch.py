from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters for topics
    input_camera_topic = '/image_raw' 
    output_image_topic = '/processed_image'  # Default output image topic from image_conversion

    return LaunchDescription([
        # Launch the usb_cam node
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe',  
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},  # Camera device
                {'image_width': 640},             # Image width
                {'image_height': 480},            # Image height
                {'pixel_format': 'yuyv'},         # Pixel format
            ],
            remappings=[
                ('/image_raw', input_camera_topic)  # Remap /image_raw to input_camera_topic
            ]
        ),

        # Launch the image_conversion node
        Node(
            package='image_conversion',  
            executable='image_conversion',  
            name='image_conversion_node',
            output='screen',
            parameters=[
                {'input_camera_topic': input_camera_topic},  
                {'output_image_topic': output_image_topic},  
            ],
            remappings=[
                ('/input_image', input_camera_topic),         
                ('/output_image', output_image_topic),        
            ]
        ),
    ])

