### sparkcan

A USB to CAN device such as a CANable is required and will need to be connected to a SPARK MAX or SPARK Flex.

Make sure to use REV Hardware Client to set the CAN ID for your controllers and modify the values to match them in the examples.

##### Installation
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

##### Running an Example

```bash
./example_control # OR ./example_status
```