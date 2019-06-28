# GPD documentation

This file contains the documentation I made while setting up the [GPD](https://github.com/rickstaa/gpd) from Atenpas.

## Dependencies

-   PCL
-   `apt-get install libgflags-dev libprotobuf-dev liblmdb-dev libleveldb-dev libsnappy-dev libatlas3-base libeigen3-dev`
-   Caffe:
        git clone https://github.com/BVLC/caffe.git
        cd caffe && mkdir build && cd build
        cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
        make && sudo make install
