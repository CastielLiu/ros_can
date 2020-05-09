## USB 权限设置，创建新的udev规则，99-usbcan.rules
1. 创建 /etc/udev/rules.d/99-usbcan.rules ,创建文件时需要sudo权限
2. 文件内容 ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="0777"
3. 重新插拔设备


