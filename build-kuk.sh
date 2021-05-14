#!/bin/bash
rm    -rf        MODULES/lib
make  -j 75 ARCH=arm64 
make ARCH=arm64 modules_install INSTALL_MOD_PATH=MODULES
