# Precision_Jig

**Precision_Jig** is a ROS 2-based application designed for vision inspection tasks in robotics.  
It runs in a containerized environment using Docker, making it easy to set up and deploy.  
A GUI is included to visualize inspection results and control system behavior.

---

## 📦 Build & Setup

> **ROS 2 Distro:** Humble  
> **OS:** Ubuntu 22.04  
> **Requirements:** Docker, Docker Compose

### 1. Clone the Repository

```bash
$ git clone <this repository>
$ cd precision_jig/ros
```

### 2. Allow X11 Access for GUI (Linux only)
```xhost +local:docker```

### 3. Build and Start the Container
```docker-compose -f "docker-compose-ros.yml" up -d ```


### (Optional) Manual Launch
```
docker exec -it ros2-container bash
cd /precision_jig_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch precision_jig amr_vision_check.launch.py
```

AND

''' docker restart <this compose container> '''

