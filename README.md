## stereo_decoder

Demo multi-threaded stereo H.264 decoder library for [ROS2 Foxy](https://docs.ros.org/en/foxy/).

### Features
* decodes [h264 messages](https://github.com/clydemcqueen/h264_image_transport) and returns
[image messages](https://index.ros.org/p/sensor_msgs/) 
* ignores timestamps, doesn't use message filters, just keeps the last stereo pair

### Example use
* subscribes to `stereo/left/image_raw/h264` and `stereo/right/image_raw/h264`
* publishes `stereo/left/image_raw` and `stereo/right/image_raw` at 1Hz

~~~
ros2 run stereo_decoder stereo_decoder_node
~~~

### Requirements

Tested on ROS2 Foxy (Ubuntu 20.04).

Requires [h264 messages](https://github.com/clydemcqueen/h264_image_transport).

Required libraries:
* libavdevice>=58
* libavformat>=58
* libavcodec>=58
* libavutil>=56
* libswscale>=5

Here's one way to install these libraries:
~~~
sudo apt install libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev libswscale-dev
~~~

Note: `rosdep` won't find keys for most of these libraries so `package.xml` declares
dependencies on `ffmpeg` and `libavdevice-dev`. Strictly speaking `ffmpeg` is not required.