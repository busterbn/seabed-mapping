from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg
from pathlib import Path
from bps_oculus.core import unpack_data_entry, polar_to_cart
import cv2


### SETUP CUSTOM MESSAGE DEFINITION FOR RawData
# Read definitions to python strings.
msg_text = """# Directly inspired by WHOI's ds_core_msgs/RawData.msg
# https://bitbucket.org/whoidsl/ds_msgs/src/master/ds_core_msgs/msg/RawData.msg
#
# Used for logging raw bytes sent to/from hardware.

std_msgs/Header header

int8 DATA_OUT=0  # Data published by the driver
int8 DATA_IN=1  # Data received by the driver

int8 direction

uint8[] data
"""

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'apl_msgs/msg/RawData'))

# Path to your ROS bag (set it correctly)
bag_path = Path('bags/blue3_2025-03-19-13-58-48.bag')
# bag_path = Path('bags/blue3_2025-03-19-12-57-10.bag')


# Define the output path (use color video in example)
output_path = bag_path.with_suffix('.mp4').with_name("oculus_" + bag_path.name)

# # Set up ffmpeg subprocess to write H.265 into MP4
# ffmpeg = subprocess.Popen([
#     'ffmpeg',
#     '-f', 'hevc',             # input is raw H.265 bitstream
#     '-framerate', '30',       # specify framerate (we know that all camera streams run at 30 fps)
#     '-i', 'pipe:0',           # read from stdin
#     '-c', 'copy',             # copy codec (no re-encode)
#     'output.mp4'              # mux to mp4
# ], stdin=subprocess.PIPE)

# Create a typestore for the matching ROS release.
typestore = get_typestore(Stores.ROS1_NOETIC)
typestore.register(add_types)

# Create reader instance and open for reading.
with Reader(bag_path) as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/oculus/raw_data':
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            ping_result, polar_image_data, new_buffer = unpack_data_entry(msg.data.tobytes())
            if ping_result is None:
                print("It was None")
                continue
            cart_image_data = polar_to_cart(polar_image_data)
            """
            EXPLAINER
            The polar_image_data and cart_image_data objects are the extracted backscatter data in polar and
            cartesian forms respectively.
            
            polar_image_data.polar_image: Backscatter image in polar form.
            polar_image_data.bearing_table: azimuth angle of each pixel in polar_image in radians
            polar_image_data.ranging_table: range for each pixel in polar_image in m
            polar_image_data.gain_table: gain applied to each range row (basically time varied gain)
            
            cart_image_data.cart_image: Backscatter image in cartesian form.
            cart_image_data.x_table: X coordinate for each pixel in cart_image in m
            cart_image_data.y_table: Y coordinate for each pixel in cart_image in m
            """
            cv2.imshow("backscatter", cart_image_data.cart_image)
            # Convert timestamp (nanoseconds) to whole seconds
            secs = int(timestamp / 1e9) - 1742392728
            min = 0
            while secs >= 60:
                min += 1
                secs -= 60
            frame_filename = f"oculus_frames/oculus_frame_{min}_min_{secs}_sec.png"
            cv2.imwrite(frame_filename, cart_image_data.cart_image)
            key = cv2.waitKey(10)
            if key & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()
