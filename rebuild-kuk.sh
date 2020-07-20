#!/bin/bash
make  ARCH=arm64 clean distclean
make  ARCH=arm64 kuk_defconfig
rm    -rf        MODULES/lib
make  -j 75 ARCH=arm64 
make ARCH=arm64 modules_install INSTALL_MOD_PATH=MODULES
