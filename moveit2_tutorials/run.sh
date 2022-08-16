# Start the container
docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/home/spaceros-user/.Xauthority:rw" -it openrobotics/moveit2_tutorials

# Launch the tutorial inside the container
#   ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true
