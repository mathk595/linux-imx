#!/bin/bash
sudo cp arch/arm64/boot/Image /media/imx6/linux_roofs/
sync
sudo rm -rf /media/imx6/linux_roofs/lib/modules/5.4.24*
sudo cp -pr MODULES/lib/modules/5.4.24* /media/imx6/linux_roofs/lib/modules/
sudo cp -p arch/arm64/boot/dts/keithkoep/kuk-trizeps8mini-ipant7.dtb /media/imx6/linux_roofs/kuk-trizeps8mini.dtb
sync
sync
