
mkdir build 2>> /dev/null
cd build && \
time cmake .. && \
time make -j6 && \
echo [OK]
