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

echo_and_run cd "/home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_states"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ilef/Euro_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ilef/Euro_ws/install/lib/python3/dist-packages:/home/ilef/Euro_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ilef/Euro_ws/build" \
    "/usr/bin/python3" \
    "/home/ilef/Euro_ws/src/robot_state_machine_behaviors/robot_state_machine_flexbe_states/setup.py" \
     \
    build --build-base "/home/ilef/Euro_ws/build/robot_state_machine_behaviors/robot_state_machine_flexbe_states" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ilef/Euro_ws/install" --install-scripts="/home/ilef/Euro_ws/install/bin"
