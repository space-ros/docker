# Start the container
docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/spaceros-user/.Xauthority:rw" -it openrobotics/moveit2_tutorials

# Inside the container, launch the tutorial
#   ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true
