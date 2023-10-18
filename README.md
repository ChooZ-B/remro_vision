RemroVision
============

image processing suite built on ROS Melodic for university project RemoteRobotics (do not download).


Usage example
-------------
within separate terminal windows execute:

    rosrun remro_vision remro_cam 0
    rosrun remro_vision crop_node image/raw image/cropped
    rosrun remro_vision binary_converter image/cropped
    rosrun remro_vision crop_dims_test
    rosrun remro_vision img_view image/binary

after selecting a motor index within *crop_dims_test*

    rosrun remro_vision angle_detector
