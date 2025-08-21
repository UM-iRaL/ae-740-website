---
layout: default
title: "Install Ubuntu 22.04"
parent: "Lab 1"
grand_parent: Labs
nav_order: 1
---

# Install Ubuntu 22.04

For this and the following labs, you need a (preferably clean) Ubuntu 22.04 LTS (Jammy Jellyfish) installation (see below). There are plenty of installation guides and tutorials on the web (and, in particular, on Youtube).

## Steps

1. Download the ISO image from [ubuntu.com](https://releases.ubuntu.com/jammy/ubuntu-22.04.4-desktop-amd64.iso)
2. Create a bootable USB stick
   - [How to create a bootable USB stick on Windows](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows)
   - [How to create a bootable USB stick on Mac OS](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-macos)
   - [How to create a bootable USB stick on Ubuntu](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu)
3. Boot from USB stick and install
   - [Install Ubuntu desktop (full erase)](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop)
   - [Install Ubuntu alongside Windows (dual boot)](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/)


{: .warning-title } 
> Warning
>
> Partitioning can be tricky if you are installing Linux for the first time. There are plenty of guides for "dual-boot Ubuntu installation" alongside both Windows and OS X. In most cases, you would first need to shrink one of your partitions (e.g., in Windows) and create an "unallocated space" which will be used during the Ubuntu installation process.
> 
> *Ask for help if you are unsure.*

## Ubuntu Setup

Once Linux is installed we need to update all the packages, to do so open a terminal (CTRL+Alt+T) and type

```bash
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake
```
