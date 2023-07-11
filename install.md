# Kinefly
Currently got it working on Ubuntu 16.04 + ROS Kinetic.

## Install ROS Kinetic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Install additional packages
```
sudo apt-get install -y python-setuptools python-scipy libdc1394-22-dev intltool gobject-introspection
sudo apt-get install -y ros-kinetic-driver-base
```

## Get aravis drivers
```
cd ~/Downloads/
wget http://mirror.accum.se/pub/GNOME/sources/aravis/0.3/aravis-0.3.7.tar.xz
tar -xf aravis-0.3.7.tar.xz
cd aravis-0.3.7
./configure
make && sudo make install
```

## Make a catkin workspace.
```
mkdir -p ~/catkin/src
cd ~/catkin/src
catkin_init_workspace
```

## Get camera_aravis ROS package
```
cd ~/catkin/src
git clone https://github.com/florisvb/camera_aravis
git checkout 497e415c5b0b9c20ac4179f8acc8ae9547799523
```

At this point, you should verify that your camera hardware is configured properly, i.e. ethernet ports & static ip's & dhcp, or firewire stuff.
Assuming you're using a Basler GigE camera, follow the general insturctions from [Basler's site](https://docs.baslerweb.com/network-configuration-(gige-cameras)#assigning-a-fixed-ip-address).
Also, don't forget to disable the firewall:
```
sudo ufw disable
```

## Get & make "libphidget"
```
cd ~/Downloads/
wget https://raw.githubusercontent.com/ungrinlab/monitor/master/libphidget.tar.gz
cd libphidget-2.1.8.20140319/
./configure 
make && sudo make install 
```

## Get and install PhidgetsPython
```
cd ~/Downloads/
wget https://raw.githubusercontent.com/ungrinlab/monitor/master/PhidgetsPython.zip
cd PhidgetsPython
sudo python setup.py install
```

## Get the LED Panels ROS node (optional).
```
cd ~/catkin/src/
git clone https://github.com/ssafarik/ledpanels
```

## Get phidgets ROS node
```
cd ~/catkin/src
git clone https://github.com/psilentp/phidgets.git
```

## Get & make the Kinefly software.
```
cd ~/catkin/src
git clone https://github.com/ssafarik/Kinefly
sudo cp ~/catkin/src/Kinefly/udev/99-phidgets.rules /etc/udev/rules.d
cd ~/catkin
catkin_make
echo "source ~/catkin/devel/setup.bash" >> ~/.bashrc
```

Think of a name for your rig (e.g. yourrigname), and tell Kinefly about it (via the RIG variable, see below).
See the Kinefly/launch directory for example rigs. You can copy one of those directories, and modify it to be your own, for example: 
```
cd ~/catkin/src/Kinefly/launch
ls
cp -R thadsrig yourrigname
echo "export RIG=yourrigname" >> ~/.bashrc
source ~/.bashrc
```

## Edit the new files as per your needs (ethernet or firewire camera, camera exposure, tracking parameters, etc).
```
cd ~/catkin/src/Kinefly/launch/yourrigname
gedit source_camera.launch
gedit params_camera.launch
gedit params_kinefly.launch
...etc...
```

## You're done!
Assuming that you got here successfully, then you  should be able to run Kinefly, for example:
```
roslaunch Kinefly main.launch  
```
