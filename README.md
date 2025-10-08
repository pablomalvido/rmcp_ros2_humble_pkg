# RMCP ROS2 HUMBLE WORKSPACE

ROS2 Humble Docker used in the AUT.841 Robot Manipulators: Modeling, Control and Programming course of Tampere University.

## Configure Docker in Windows
1. Download and install Docker Desktop: https://docs.docker.com/desktop/setup/install/windows-install/ 

2. After installation, it may ask you to update wsl. In that case, write this command in the terminal: 

```
wsl --update
```
3. Download xlaunch for windows: https://sourceforge.net/projects/vcxsrv/

4. Set up xlaunch: https://www.youtube.com/watch?v=qWuudNxFGOQ

5. Download Visual Studio Code: https://code.visualstudio.com/download

6. Download 'Docker' and 'Dev Containers' extensions in Visual Studio Code

## Configure Docker in Linux
**Install Docker:**

1. Update packages
```
sudo apt update
sudo apt upgrade -y
```

2. Install prerequisites
```
sudo apt install -y apt-transport-https ca-certificates curl software-properties-common gnupg lsb-release
```

3. Add Docker’s official GPG key
```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
```

4. Add the Docker repository
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

5. Install Docker Engine
```
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

6. Enable and start Docker
```
sudo systemctl enable docker
sudo systemctl start docker
```

7. Run Docker without sudo
```
sudo usermod -aG docker $USER
newgrp docker
```

**Install Visual Studio Code:**
```
sudo apt update
sudo apt install -y wget gpg apt-transport-https software-properties-common
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install -y code
```

## Setup in Windows
**1. Clone the Repository**

Clone the repository in a folder:
```
git clone https://github.com/pablomalvido/rmcp_ros2_humble_pkg.git
```

**2. Open Visual Studio Code ...**

Then, open the folder ```rmcp_ros2_humble_pkg``` in Visual Studio Code

**3. Choose Reopen in container when prompted**

The container will be built automatically, as required.

**4. Enable the use of your display:**

Follow these steps (check video from 17:48 to 18:40): https://youtu.be/qWuudNxFGOQ?si=XCha-tFMAlJTWY0P&t=1069

**5. Open a terminal and build the workspace:**

```
colcon build --symlink-install --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**6. Source the built workspace environment:**

```
source install/setup.bash
```

## Setup in Linux
**1. Clone the Repository (Linux branch)**

Clone the repository in a folder:
```
git clone -b linux https://github.com/pablomalvido/rmcp_ros2_humble_pkg.git
```

**2. Open Visual Studio Code ...**

Then, open the folder ```rmcp_ros2_humble_pkg``` in Visual Studio Code

**3. Choose Reopen in container when prompted**

The container will be built automatically, as required.

**4. Enable the use of your display:**

Run this command on your host:

```
sudo xhost +local:docker
```

**5. Open a terminal and build the workspace:**

```
colcon build --symlink-install --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**6. Source the built workspace environment:**

```
source install/setup.bash
```

## Test (Windows and Ubuntu)

After installation and setup, you can test if it works. Inside every package, you will find a README file with instructions to run all the exercises. For testing you can use the most simple package, topics_pkg. Open this package and follow the instructions.

**DO NOT FORGET TO SOURCE THE WORKSPACE IN EVERY TERMINAL YOU USE**
```
source install/setup.bash
```
