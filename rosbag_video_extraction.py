from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
import subprocess
from pathlib import Path

# Path to your ROS bag (set it correctly)
bag_path = Path('bags/blue3_2025-03-19-13-58-48.bag')
# Define the output path (use color video in example)
output_path = bag_path.with_suffix('.mp4').with_name("rgb_" + bag_path.name)

# Set up ffmpeg subprocess to write H.265 into MP4
ffmpeg = subprocess.Popen([
    'ffmpeg',
    '-f', 'hevc',             # input is raw H.265 bitstream
    '-framerate', '30',       # specify framerate (we know that all camera streams run at 30 fps)
    '-i', 'pipe:0',           # read from stdin
    '-c', 'copy',             # copy codec (no re-encode)
    'output.mp4'              # mux to mp4
], stdin=subprocess.PIPE)

# Create a typestore for the matching ROS release.
typestore = get_typestore(Stores.ROS1_NOETIC)

# Create reader instance and open for reading.
with Reader(bag_path) as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/oak_d_lite/rgb/image_color/h265':
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            ffmpeg.stdin.write(msg.data.tobytes())

# Finalize ffmpeg
ffmpeg.stdin.close()
ffmpeg.wait()
