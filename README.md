# Ladybug5 ROS Driver

ROS driver for the FLIR Ladybug5 360° camera. Supports both native Ubuntu 20.04 installation and Docker deployment.

## Repository Structure
```
catkin_ladybug_ws/
├── src/
│   └── ladybug_driver/     # ROS package
├── docker/                 # Docker support files
├── Dockerfile             # Docker build definition
└── docker-compose.yml     # Docker compose config
```

## Installation Options

### Option 1: Native Installation (Ubuntu 20.04)

1. Clone the repository:
```bash
git clone https://github.com/Ali-Rizvi-1/catkin_ladybug_ws.git
cd catkin_ladybug_ws
```

2. Install ROS Noetic (if not already installed):
```bash
# Follow instructions at: http://wiki.ros.org/noetic/Installation/Ubuntu
```

3. Install Ladybug SDK:
```bash
# Obtain ladybug-1.20.0.79_amd64.deb from FLIR
sudo dpkg -i ladybug-1.20.0.79_amd64.deb
```

4. Setup udev rules:
```bash
sudo groupadd -f flirimaging
sudo usermod -aG flirimaging $USER
sudo bash -c 'echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"3800\", MODE=\"0664\", GROUP=\"flirimaging\"" > /etc/udev/rules.d/40-pgr-ladybug.rules'
sudo udevadm control --reload-rules && sudo udevadm trigger
```

5. Build the workspace:
```bash
catkin_make
source devel/setup.bash
```

#### Usage

6. Launch the camera node:
```bash
roslaunch ladybug_driver ladybug.launch
```


### Option 2: Docker Installation and Setup

1. Build and enter the container:
```bash
docker-compose run --rm ladybug /bin/bash
```

2. Inside the container, install the SDK:
```bash
cd /ladybug_sdk
sudo dpkg -i ladybug-1.20.0.79_modified.deb
```

3. Setup udev rules and groups:
```bash
# Create flirimaging group
sudo groupadd -f flirimaging
sudo usermod -aG flirimaging ladybug_user

# Create udev rules
sudo bash -c 'echo "ATTRS{idVendor}==\"1e10\", ATTRS{idProduct}==\"3800\", MODE=\"0664\", GROUP=\"flirimaging\"" > /etc/udev/rules.d/40-pgr-ladybug.rules'
sudo bash -c 'echo "KERNEL==\"raw1394\", MODE=\"0664\", GROUP=\"flirimaging\"" >> /etc/udev/rules.d/40-pgr-ladybug.rules'
sudo bash -c 'echo "KERNEL==\"video1394*\", MODE=\"0664\", GROUP=\"flirimaging\"" >> /etc/udev/rules.d/40-pgr-ladybug.rules'
sudo bash -c 'echo "SUBSYSTEM==\"firewire\", GROUP=\"flirimaging\"" >> /etc/udev/rules.d/40-pgr-ladybug.rules'

# Reload udev rules
sudo service udev restart
```

4. Build the ROS package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

5. Launch the node:
```bash
roslaunch ladybug_driver ladybug.launch
```

### Opening Additional Terminals

To open additional terminals for the running container:

1. Find the container ID:
```bash
docker ps
```

2. Open a new terminal in the running container:
```bash
docker exec -it <container_id> /bin/bash
```

## Published Topics

The driver publishes images to:
- `/ladybug/camera0/image_raw`
- `/ladybug/camera1/image_raw`
- `/ladybug/camera2/image_raw`
- `/ladybug/camera3/image_raw`
- `/ladybug/camera4/image_raw`
- `/ladybug/camera5/image_raw`

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scale` | 100 | Image scale percentage |
| `jpeg_percent` | 80 | JPEG quality |
| `framerate` | 10 | Camera framerate |
| `use_auto_framerate` | false | Enable auto framerate |
| `shutter_time` | 0.5 | Shutter time (seconds) |
| `use_auto_shutter_time` | true | Enable auto shutter |
| `gain_amount` | 10 | Gain amount |
| `use_auto_gain` | true | Enable auto gain |

## Troubleshooting

1. **Camera Not Detected**
   - Check USB connection
   - Verify udev rules are loaded
   - Ensure user is in flirimaging group

2. **Permission Issues**
   - Log out and back in after group changes
   - Check device permissions: `ls -l /dev/bus/usb`

3. **Docker Issues**
   - Ensure Docker service is running
   - Check USB device visibility: `lsusb`

## License

[MIT License](LICENSE)

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request