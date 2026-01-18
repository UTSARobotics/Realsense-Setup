# Realsense-Setup

## Kira KV260 Initial Setup 

Follow the instructions detailed here: https://xilinx.github.io/kria-apps-docs/kv260/2022.1/linux_boot/ubuntu_22_04/build/html/docs/sdcard.html

## Realsense D345 Setup

NOTE: This setup is for Linux only. 

Download the precompiled SDK here: https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md and follow the instructions for setup. 

## Running realsense-aruco.py

Download the python dependencies for the Realsense viewer https://github.com/realsenseai/librealsense/blob/master/readme.md#install.

As well as, the OpenCV dependencies https://docs.opencv.org/4.x/db/dd1/tutorial_py_pip_install.html.

Finally, connect the realsense viewer to your machine and simply run
```python
python3 realsense-aruco.py
```

