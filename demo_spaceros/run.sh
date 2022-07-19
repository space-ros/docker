docker run --rm --gpus all --net=host -e DISPLAY=$DISPLAY --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it openrobotics/spaceros-demo
