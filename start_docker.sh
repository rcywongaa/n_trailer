xhost +local:root; docker run -v $PWD:/workspace -i -e DISPLAY \
-e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
--privileged -t drake bash; xhost -local:root
