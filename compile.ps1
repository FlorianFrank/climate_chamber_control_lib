IF not exist "./tmp" (mkdir "./tmp")
pushd "tmp"
cmake -GNinja ..  -DINSTALL_DIR=%cd%/../bin
ninja
ninja install
popd
