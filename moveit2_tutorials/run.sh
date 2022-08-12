# Launch the container
docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it openrobotics/moveit2_tutorials

# Inside the container:

# Run xauth add (get the values from the host system via 'xauth list')
#   xauth add <hostname>/unix:1  MIT-MAGIC-COOKIE-1  <cookie-value>

# Launch the tutorial
#   ros2 launch moveit2_tutorials demo.launch.py rviz_tutorial:=true
