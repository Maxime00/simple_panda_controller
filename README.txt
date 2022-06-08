## Simple Panda Controller

Barebones structure for panda controller using docker and franka-lightweight-interface

# Deployment Instructions
./build-server.sh -s
aica-docker connect simple-panda-controller-something

cd src/simple_controllers/python
python3 cartesian_impedance_controller.py

# deployment
aica-docker interactive simple-panda-controller:latest -u ros2 --net host --no-hostname


To be used for Ahalya's learning safety margin project


TODO :
figure out minimal control rate
add launch script
make branch for ahalya's project with rviz
redo structure to be better template (add controllers, put robot interface elsewhere)