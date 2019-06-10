#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/src/joystick_drivers/wiimote"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/install/lib/python2.7/dist-packages:/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/build" \
    "/usr/bin/python" \
    "/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/src/joystick_drivers/wiimote/setup.py" \
    build --build-base "/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/build/joystick_drivers/wiimote" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/install" --install-scripts="/home/nvidia/Wokspace/umfordavmodelcar_ws/ros_ws/joy_node_ws/install/bin"
