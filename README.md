#### Disclaimer
This project is not affiliated with, endorsed by, or in any way connected to REV Robotics. All product names, logos, and brands are property of their respective owners. Any references to REV Robotics products, such as the SPARK MAX or SPARK Flex, are purely for identification purposes and do not imply any endorsement or sponsorship.

#### Setup
A SocketCAN USB adapter such as a [CANable](https://canable.io/) is required and will need to be connected to a [Power Distribution Hub](https://www.revrobotics.com/rev-11-1850/). The Power Distribution Hub will need to be connected to a [SPARK MAX](https://www.revrobotics.com/rev-11-2158/) and/or [SPARK Flex](https://www.revrobotics.com/rev-11-2159/).

Make sure to use REV Hardware Client to set the CAN ID for your controllers and modify the values to match them in the examples.

#### Installation
```bash
sudo apt update
sudo apt upgrade
sudo apt install git cmake

sudo add-apt-repository ppa:graysonarendt/sparkcan
sudo apt update
sudo apt install sparkcan

git clone https://github.com/grayson-arendt/sparkcan-examples.git
cd sparkcan-examples/

chmod +x canableStart.sh
./canableStart.sh

mkdir build
cd build
cmake ..
make
```

#### Running an Example

```bash
./example_control # OR ./example_status
```
