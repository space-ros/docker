docker run --rm --net=host -e DISPLAY=$DISPLAY  --device=/dev/dri:/dev/dri --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -it openrobotics/riscv_rtems:latest
