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

echo_and_run cd "/home/glab/Desktop/fetch_test/src/geometry2/tf2_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/glab/Desktop/fetch_test/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/glab/Desktop/fetch_test/install/lib/python2.7/dist-packages:/home/glab/Desktop/fetch_test/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/glab/Desktop/fetch_test/build" \
    "/usr/bin/python2" \
    "/home/glab/Desktop/fetch_test/src/geometry2/tf2_py/setup.py" \
     \
    build --build-base "/home/glab/Desktop/fetch_test/build/geometry2/tf2_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/glab/Desktop/fetch_test/install" --install-scripts="/home/glab/Desktop/fetch_test/install/bin"
