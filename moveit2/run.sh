# Start the container
docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/spaceros-user/.Xauthority:rw" -it openrobotics/moveit2

# Inside the container, launch the demo
#   ros2 launch mars_rover mars_rover.launch.py

