---
layout: default
title: "Installing ROS"
parent: "Lab 2"
grand_parent: Labs
nav_order: 1
---

# Installing ROS
{: .fs-9 .no_toc}

![ROS](https://www.generationrobots.com/blog/wp-content/uploads/2016/03/Logo-ROS-Robot-Operating-System1.jpg)

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Getting started with ROS

The Robot Operating System (ROS) is a crucial middleware (a.k.a. collection of software packages) that enables roboticists all over the world to implement their algorithms in a clean and modular fashion and share their work effectively with the community.

In addition, it is at the very core of our class, so we'd better start playing with it!

## Installing ROS

We will be using ROS 2 "Humble" in this class. If you search for ROS-related content online, you will find references to ROS 1 and ROS 2. ROS 1 has recently reached its end-of-life, and you will want to make sure you are looking at the ROS 2 version of any documentation that you find.

By now, you should have a working (preferably fresh) install of Ubuntu 22.04 and have become accustomed with the basics of Linux, Git and C++. The most efficient way to install ros is through the Debian (binary) packages.

To install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html){:target="_blank"} on it, we will follow the [official guide](https://docs.ros.org/en/humble/Installation.html) to install the **Desktop Install option**.

### Setup repositories

First we will enable the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu)

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Next we will setup the keys (install `curl` if you haven't already via `sudo apt install curl`)

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

and then add the repository to the sources list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Installation

Before installing ROS we need to update the apt index

```bash
sudo apt update
```

We also need to make sure that all packages on the system are up to date. This step may take some time.

```bash
sudo apt upgrade
```

Now let's install ROS 🤖

```bash
sudo apt install ros-humble-desktop '~nros-humble-rqt*'
```

### Environment setup

It's convenient if the ROS environment variables are automatically loaded as soon a new shell is launched, let's edit `~/.bashrc` to do so

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

{: .warning-title }
> **Keep in mind.**
> 
> If you use a different shell (such as zsh), make sure you configure your shell to source the correct setup file!

By default, ROS 2 can connect to nodes on other computers on the same network. As we do not want this behavior, we will restrict ROS to only connect to nodes on the same computer:

```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### Validating the Installation

We will now run some example code to make sure everything installed correctly.

In one terminal, source the setup file and then run a C++ talker:

```bash
ros2 run demo_nodes_cpp talker
```

In another terminal, run

```bash
ros2 run demo_nodes_py listener
```

You should see the talker saying that it's Publishing messages and the listener saying I heard those messages. This verifies both the C++ and Python APIs are working properly.

Next run

```bash
printenv | grep -i ROS
```

and make sure `ROS_DISTRO`, `ROS_VERSION`, and `ROS_PYTHON_VERSION` are set:

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### Dependencies for building packages

Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool build-essential python3-colcon-common-extensions
```

There are several quality of life improvements related to autocomplete that you may want to add add the end of your `.bashrc`

```bash
eval "$(register-python-argcomplete3 ros2)"             # adds tab-completion for ros2 commands
eval "$(register-python-argcomplete3 colcon)"           # adds tab-completion for colcon commands
source /usr/share/colcon_cd/function/colcon_cd.sh       # enables colcon_cd package_name
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
export _colcon_cd_root=/opt/ros/humble/                 # sets the colcon_cd root
```

### Initialization

Before using ROS, we need to initialize `rosdep`.

```bash
sudo rosdep init
rosdep update
```