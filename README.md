# Autonomous_Golf_Cart
Perception, Localization, Motion Planner, and Control for Golf Cart made by Engineering Physics Final Project Students est. 2018

## Before you run the whole thing

1. Install Ubuntu 20.04 LTS (This Repo will not run on Ubuntu 22.04)

2. [Install ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

3. create ROS workspace
follow ['ROS Tutorial 1.1 1. Installing and Configuring Workspace'](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
You probably need to follow the whole beginner level tutorials before proceeding to the next step

4. Clone the repository INSIDE the src folder of your BUILT workspace

'git clone https://github.com/mpandoko/Autonomous_Golf_Cart.git'

The directory path after you clone should look like this (Golf_cart_ws is the name of mp's workspace)

```bash
~/Golf_cart_ws/src/Autonomous_Golf_Cart$ 
```

Always remember to source your environment setup everytime you open new terminals

```bash
source devel/setup.bash
```

5. Make sure that you fulfill all the requirements: Python 3.8 or later with all [requirements.txt](https://github.com/mpandoko/Autonomous_Golf_Cart/blob/main/requirements.txt) dependencies installed, including torch>=1.7. To install, run:

`pip install -r requirements.txt`

6. Install all dependencies and build the packages

```bash
In your workspace (If you were still in the src folder then `cd ..`), 

rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

7. Try to launch the system (do it in your ws folder, NOT the Autonomous_Golf_Cart folder)

```bash
roslaunch golfi ukf_ule.launch #ule, atwin
roslaunch golfi tes_sensor.launch #ule, atwin
roslaunch pkg_ta vD_control-2d-ukf-localization.launch #adam, pras
rosrun persepsi track.py #mp, sat
rosrun behaviour_tree trees.py #mp, sat
```

kalau ada error 'file not executable':
```bash
	roscd pkg_ta
	cd /nodes
	chmod +x control_2d_ukf_localization.py
```

## Simulators

Two different simulators, both are optional, not need to install to run golfcar:

1. Mvsim (light but ugly, the installation steps might be wrong!!) 

```bash
sudo add-apt-repository ppa:joseluisblancoc/mrpt   # develop branch
sudo apt install libmrpt-dev mrpt-apps
sudo apt-get install ros-noetic-mrpt-bridge
```

[Install all requirements](https://mvsimulator.readthedocs.io/en/latest/install.html)

```bash
sudo apt-get install libmrpt-dev
```

Try to run

```bash
roslaunch mvsim mvsim_demo_1robot.launch
rosrun mvsim converter.py
```

2. Carla 
install carla
	follow the installation guide
		extract the .tar.gz file
		install python dependencies for client
	install OpenGL3
	run with -opengl3 flag
	
## Tmux and Tmuxinator

Also optional but trust me this will save your time and sanity

1. Tmux

[https://linuxize.com/post/getting-started-with-tmux/](https://linuxize.com/post/getting-started-with-tmux/)
[https://tmuxcheatsheet.com/](https://linuxize.com/post/getting-started-with-tmux/)

2. Tmuxinator

Install

```bash
sudo apt-get install -y tmuxinator 
```

[guide](https://github.com/tmuxinator/tmuxinator)
Copy all the files from etc/tmuxinator in this repo to ~/.config/tmuxinator

## Running the whole thing

1. Real Golf Cart

Urutan nyolok USB ke dongle:
1. GNSS (Cek 'ls /dev/tty' harusnya muncul ttyACM0)
2. IMU (Cek 'ls /dev/tty' harusnya muncul ttyACM0 dan ttyACM1)
3. Arduino mega (Cek 'ls /dev/USB' harusnya muncul ttyUSB0)

Kamera terserah kapan, jangan dicolok ke dongle, harus langsung ke laptop

I suggest you use tmux and skip point a

a. If you are not using tmuxinator, run each file on each terminal (you need at least 8 terminals)  (run it in your ws) (don't forget to source everytime you open new terminal)

```bash
roslaunch pkg_ta vD_control-2d-ukf-localization.launch
```

```bash
rosrun persepsi track.py
```

```bash
rosrun behaviour_tree live_plot.py
```

```bash
rosrun behaviour_tree trees.py
```

```bash
sudo chmod a+rw /dev/ttyACM0
roslaunch golfi vD_gnss.launch
```

```bash
sudo chmod a+rw /dev/ttyACM1
roslaunch golfi vD_imu.launch
```

```bash
roslaunch golfi vD_ukf_ule.launch
```

Do this last, when you are ready to launch the golf car
```bash
sudo chmod a+rw /dev/ttyUSB0
roslaunch golfi vD_mega.launch
```

b. If you are using tmuxinator, you'll only need 3 terminals (NO NEED to source) (run it from your home directory '~')

```bash
tmuxinator loc
```

```bash
tmuxinator gc
```

```bash
tmuxinator mega

# when you are ready to launch the golf car, in the tmux mega session:
roslaunch golfi vD_mega.launch
```

See how superior tmux is?
Learn how to edit tmuxinator (.yml) files to add more launch files

2. MVSIM
```bash
tmuxinator gc
```

```bash
tmuxinator sim

# when you are ready to launch the golf car, in the tmux sim session:
rosrun mvsim converter.py
```

## Papers
Papers (Indonesia): https://drive.google.com/drive/u/0/folders/1Ggm_oNL2Gx75hSiE9dpRdmb3VvBJKeMs
