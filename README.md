# RoboticsCPP
Robotics project using C++

# Environment
- Ubuntu 22.04
- ROS2 Humble
- OpenCV 4.8.1

# Description
모바일 로봇이 특정 목적지까지 객체를 탐지하며 회피 기동
Robotics example using C++
# How to Usage

### Docker version
```bash
xhost +local:docker

git clone https://github.com/ans2568/RoboticsCPP.git
cd RoboticsCPP
docker build -t roboticscpp .
docker run -it -d --name=roboticscpp --network=host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v /dev/shm:/dev/shm --ipc=host roboticscpp
```

### Local version
```bash
# dependency ROS2 humble, OpenCV 4.x.x, PCL
git clone https://github.com/ans2568/RoboticsCPP.git
cd RoboticsCPP
mkdir build
cd build
cmake ..
make -j
./MAIN
```
# TODO List

- [ ] Create module class
    - [ ] Camera
    - [ ] LiDAR
    - [ ] Navigation
    
- [ ] Read camera and LiDAR sensor data
- [ ] Navigate to destination by using A* algorithm
- [ ] Visualize mobile robot(turtlebot3)