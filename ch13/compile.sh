
mkdir build 2>> /dev/null
cd build && \
cmake .. && \
make -j6 && \
echo [OK]
