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

echo_and_run cd "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_ros/fetch_calibration"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sebastian/git/FetchRobotControl-41014/packages/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sebastian/git/FetchRobotControl-41014/packages/install/lib/python2.7/dist-packages:/home/sebastian/git/FetchRobotControl-41014/packages/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sebastian/git/FetchRobotControl-41014/packages/build" \
    "/usr/bin/python2" \
    "/home/sebastian/git/FetchRobotControl-41014/packages/src/fetch_ros/fetch_calibration/setup.py" \
     \
    build --build-base "/home/sebastian/git/FetchRobotControl-41014/packages/build/fetch_ros/fetch_calibration" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sebastian/git/FetchRobotControl-41014/packages/install" --install-scripts="/home/sebastian/git/FetchRobotControl-41014/packages/install/bin"
