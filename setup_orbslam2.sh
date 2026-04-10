#!/bin/bash
# Builds ORB-SLAM2 (patched for OpenCV 4.x) in ~/orb_slam2
# Usage:  bash setup_orbslam2.sh
set -euo pipefail

SLAM_DIR="$HOME/orb_slam2"
PANGOLIN_DIR="$HOME/Pangolin"
JOBS=$(nproc)

die() { echo "ERROR: $*" >&2; exit 1; }

echo "=== [1/5] Installing system dependencies ==="
sudo apt-get update -qq
sudo apt-get install -y \
    cmake git build-essential \
    libeigen3-dev libboost-all-dev \
    libssl-dev \
    libgl1-mesa-dev libglew-dev \
    libepoxy-dev \
    libopencv-dev \
    libpython3-dev python3-dev

echo "=== Locating OpenCV cmake config ==="
OPENCV_DIR=""
for candidate in \
    /usr/lib/x86_64-linux-gnu/cmake/opencv4 \
    /usr/lib/aarch64-linux-gnu/cmake/opencv4 \
    /usr/share/opencv4 \
    /usr/local/lib/cmake/opencv4; do
    if [ -f "$candidate/OpenCVConfig.cmake" ]; then
        OPENCV_DIR="$candidate"
        break
    fi
done
# Fallback: search
if [ -z "$OPENCV_DIR" ]; then
    OPENCV_CMAKE=$(find /usr -name "OpenCVConfig.cmake" 2>/dev/null | head -1)
    [ -n "$OPENCV_CMAKE" ] && OPENCV_DIR=$(dirname "$OPENCV_CMAKE")
fi
[ -n "$OPENCV_DIR" ] || die "OpenCVConfig.cmake not found after installing libopencv-dev. Run: dpkg -l libopencv-dev"
echo "OpenCV cmake config dir : $OPENCV_DIR"
# CMAKE_PREFIX_PATH needs the grandparent of the cmake/<pkg> dir (i.e. /usr/lib/x86_64-linux-gnu)
OPENCV_PREFIX=$(dirname "$(dirname "$OPENCV_DIR")")
echo "OpenCV cmake prefix     : $OPENCV_PREFIX"

echo "=== [2/5] Building Pangolin ==="
if [ -f /usr/local/lib/libpangolin.so ] || [ -f /usr/local/lib/libpangolin.a ] || \
   find /usr/local/lib -name "libpangolin*" 2>/dev/null | grep -q .; then
    echo "Pangolin already installed, skipping."
else
    [ -d "$PANGOLIN_DIR" ] || git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git "$PANGOLIN_DIR"
    mkdir -p "$PANGOLIN_DIR/build" && cd "$PANGOLIN_DIR/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF
    make -j"$JOBS"
    sudo make install
    sudo ldconfig
    echo "Pangolin built."
fi

echo "=== [3/5] Cloning ORB-SLAM2 ==="
[ -d "$SLAM_DIR" ] || git clone https://github.com/raulmur/ORB_SLAM2.git "$SLAM_DIR"
cd "$SLAM_DIR"

echo "=== [4/5] Patching ==="

# C++14
grep -q "std=c++14" CMakeLists.txt || \
    sed -i 's/-Wall -O3 -march=native"/-Wall -O3 -march=native -std=c++14"/' CMakeLists.txt

# Deprecated OpenCV constants
find src include Examples -name "*.cc" -o -name "*.h" -o -name "*.cpp" | while read -r f; do
    sed -i \
        -e 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' \
        -e 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' \
        -e 's/CV_LOAD_IMAGE_COLOR/cv::IMREAD_COLOR/g' \
        -e 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' \
        -e 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g' \
        -e 's/CV_RGB2GRAY/cv::COLOR_RGB2GRAY/g' \
        -e 's/CV_AA/cv::LINE_AA/g' \
        -e 's/CV_FILLED/cv::FILLED/g' \
        "$f"
done

# LoopClosing.h typedef
sed -i \
    's/std::map<KeyFrame\*,g2o::Sim3,std::less<KeyFrame\*>,Eigen::aligned_allocator<std::pair<const KeyFrame\*, g2o::Sim3> > >/KeyFrameAndPose/g' \
    include/LoopClosing.h || true
grep -q "typedef.*KeyFrameAndPose" include/LoopClosing.h || \
    sed -i '/#ifndef LOOPCLOSING_H/a typedef std::map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3>>> KeyFrameAndPose;' \
    include/LoopClosing.h

# Inject OpenCV_DIR into both CMakeLists before find_package — use Python for reliable replacement
python3 - "$OPENCV_DIR" <<'PYEOF'
import sys, re, pathlib, subprocess

opencv_dir = sys.argv[1]

# Get include dirs and libs from pkg-config (guaranteed to work if libopencv-dev is installed)
inc = subprocess.check_output(['pkg-config', '--cflags-only-I', 'opencv4']).decode().strip()
inc_dirs = ';'.join(p[2:] for p in inc.split() if p.startswith('-I'))
libs_raw = subprocess.check_output(['pkg-config', '--libs-only-l', 'opencv4']).decode().strip()
libs = ';'.join(p[2:] for p in libs_raw.split() if p.startswith('-l'))
lib_dirs_raw = subprocess.check_output(['pkg-config', '--libs-only-L', 'opencv4']).decode().strip()
lib_dirs = ';'.join(p[2:] for p in lib_dirs_raw.split() if p.startswith('-L'))

print(f"  OpenCV include dirs: {inc_dirs}")
print(f"  OpenCV libs: {libs}")

# Replacement block that bypasses find_package entirely
pkgconfig_block = f'''# OpenCV via pkg-config (patched by setup_orbslam2.sh)
set(OpenCV_FOUND TRUE)
set(OpenCV_INCLUDE_DIRS "{inc_dirs}")
set(OpenCV_LIBS "{libs}")
link_directories("{lib_dirs}")
'''

for path in [
    pathlib.Path("Thirdparty/DBoW2/CMakeLists.txt"),
    pathlib.Path("CMakeLists.txt"),
]:
    txt = path.read_text()
    # Remove any previous injection
    txt = re.sub(r'# OpenCV via pkg-config.*?link_directories\([^\)]*\)\n', '', txt, flags=re.DOTALL)
    txt = re.sub(r'set\(OpenCV_DIR[^\n]*\)\n', '', txt)
    # Replace the entire find_package(OpenCV...) block with our pkg-config block.
    # DBoW2 has a nested if structure; we count endif() calls to find the full extent.
    import re as _re
    start = _re.search(r'find_package\s*\(\s*OpenCV', txt)
    if start:
        i = start.start()
        depth = 0
        j = i
        while j < len(txt):
            if txt[j:].startswith('if(') or txt[j:].startswith('if ('):
                depth += 1
            elif txt[j:].startswith('endif()') or txt[j:].startswith('endif ()'):
                depth -= 1
                if depth == 0:
                    j = txt.index('\n', j) + 1
                    break
            j += 1
        txt = txt[:i] + pkgconfig_block + txt[j:]
    path.write_text(txt)
    print(f"  Patched {path}")
PYEOF

# Save per-frame trajectory
grep -q "SaveTrajectoryTUM" Examples/Monocular/mono_tum.cc || \
    sed -i \
        's/SLAM\.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory\.txt")/SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");\n    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt")/' \
        Examples/Monocular/mono_tum.cc

echo "=== [5/5] Building ==="

CMAKE_OPENCV_ARGS=(
    -DCMAKE_BUILD_TYPE=Release
    -DOpenCV_DIR="$OPENCV_DIR"
    -DCMAKE_PREFIX_PATH="$OPENCV_PREFIX"
)
echo "cmake OpenCV args: ${CMAKE_OPENCV_ARGS[*]}"

# DBoW2
if [ ! -f "$SLAM_DIR/Thirdparty/DBoW2/lib/libDBoW2.so" ]; then
    cd "$SLAM_DIR/Thirdparty/DBoW2"
    rm -rf build && mkdir build && cd build
    cmake .. "${CMAKE_OPENCV_ARGS[@]}"
    make -j"$JOBS"
else
    echo "DBoW2 already built."
fi

# g2o
if [ ! -f "$SLAM_DIR/Thirdparty/g2o/lib/libg2o.so" ]; then
    cd "$SLAM_DIR/Thirdparty/g2o"
    rm -rf build && mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j"$JOBS"
else
    echo "g2o already built."
fi

cd "$SLAM_DIR"
[ -f Vocabulary/ORBvoc.txt ] || (cd Vocabulary && tar -xf ORBvoc.txt.tar.gz)

rm -rf build && mkdir build && cd build
cmake .. "${CMAKE_OPENCV_ARGS[@]}"
make -j"$JOBS"

echo ""
echo "=== Done! ==="
echo "Binary: $SLAM_DIR/Examples/Monocular/mono_tum"
echo "Next:   cd /mnt/c/Users/Nybo/Documents/pose-validation && python3 run_orbslam_benchmark.py"
