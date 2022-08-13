# Launch the container
docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/home/spaceros-user/.Xauthority:rw" -it openrobotics/spaceros-demo

# Launch the demo inside the container
#   ros2 launch mars_rover mars_rover.launch.py
