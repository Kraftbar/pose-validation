V=/home/nybo/github/pose-validation/external/vio
export VROOT=$V/deps/root/usr
export OCV=$V/deps/opencv
export CMAKE_PREFIX_PATH=$VROOT:$OCV
export LD_LIBRARY_PATH=$VROOT/lib/x86_64-linux-gnu:$VROOT/lib/x86_64-linux-gnu/openblas-pthread:$OCV/lib:/home/nybo/github/pose-validation/external/candidates/deps/root/usr/lib/x86_64-linux-gnu
export PATH=/tmp/vio-venv/bin:/home/nybo/.local/bin:$PATH
export CPATH=$VROOT/include:$VROOT/include/x86_64-linux-gnu:$VROOT/include/eigen3:$OCV/include/opencv4
export LIBRARY_PATH=$VROOT/lib/x86_64-linux-gnu:$VROOT/lib/x86_64-linux-gnu/openblas-pthread:$OCV/lib
