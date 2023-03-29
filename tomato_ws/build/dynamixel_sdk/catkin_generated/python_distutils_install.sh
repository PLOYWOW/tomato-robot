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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hayashi/worksp/tomato_ws/src/DynamixelSDK/ros/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hayashi/worksp/tomato_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hayashi/worksp/tomato_ws/install/lib/python3/dist-packages:/home/hayashi/worksp/tomato_ws/build/dynamixel_sdk/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hayashi/worksp/tomato_ws/build/dynamixel_sdk" \
    "/usr/bin/python3" \
    "/home/hayashi/worksp/tomato_ws/src/DynamixelSDK/ros/dynamixel_sdk/setup.py" \
     \
    build --build-base "/home/hayashi/worksp/tomato_ws/build/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hayashi/worksp/tomato_ws/install" --install-scripts="/home/hayashi/worksp/tomato_ws/install/bin"
