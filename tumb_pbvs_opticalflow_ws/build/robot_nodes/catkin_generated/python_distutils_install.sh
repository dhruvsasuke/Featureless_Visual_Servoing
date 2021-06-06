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

echo_and_run cd "/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/src/robot_nodes"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/install/lib/python2.7/dist-packages:/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/build" \
    "/usr/bin/python2" \
    "/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/src/robot_nodes/setup.py" \
     \
    build --build-base "/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/build/robot_nodes" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/install" --install-scripts="/home/krishna/projects/RP2/tumb_pbvs_opticalflow_ws/install/bin"
