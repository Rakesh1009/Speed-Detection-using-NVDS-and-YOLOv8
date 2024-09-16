# Speed detection of an object using NVIDIA DeepStream & Ultralytics Yolov8

Calculate the speed of a vehicle in real time!

![FPS](resources/yolov8.gif)

## Repository setup

This is a straightforward step, however, if you are new to git, I recommend glancing through the steps.

First, install git

```sh
sudo apt install git
```

Next, clone the repository

```sh
https://github.com/Rakesh1009/Speed-Detection-using-NVDS-and-YOLOv8
```

## Download the model files

You can run the `download-models.sh` script to download `onnx` yolov8 models, these files are parsed by [this parser](custom_parsers/nvds_customparser_yolov8).
```sh
cd models
chmod 0777 download-models.sh
./download-models.sh
```

However, I used the [custom parser](custom_parsers/nvdsinfer_custom_impl_Yolo) , whose `onnx` files need to be generated by using this [method](https://github.com/marcoslucianops/DeepStream-Yolo/blob/master/docs/YOLOv8.md). Using this method is recommended, it is the default parser in the [Makefile](Makefile).

## Docker image build
- This repository is tested and used with `docker`. For docker setup with `nvidia`, [click here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
- I have tested by hosting my webcam as an RTSP server, I have left the comments of those steps in the files, so you will find traces of them throughout the [Dockerfile](docker/dgpu/Dockerfile) and the [compose](docker/dgpu/compose.yaml) file.

### 1. Build the image

```sh
cd docker/dgpu

docker compose build
```

### 2. Start a container

```sh
xhost +

docker compose up
```
I have used docker compose to make the process easier.

## Running the Application

- Just starting the container using docker compose up will run the container and a script [start.sh](start.sh).
- You can change it, it has all the steps necessary to run the application, compiling the application using make, executing the generated executable.
- Before you do this though, create a file called `inputsources.txt` and paste the path of videos or rtsp url, this application also supports webcam devices which have to be syntaxed as follows.

```sh
file:///home/astr1x/Videos/sample.mp4
rtsp://admin:admin%40123@192.168.1.1:554/stream
camera:///dev/video0
```

## Citations

* [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)