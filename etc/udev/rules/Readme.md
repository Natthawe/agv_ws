## Assign a static USB port on Linux

    lsusb

    sudo dmesg | grep ttyUSB

    sudo udevadm info --name=/dev/ttyACMx --attribute-walk >> ACM_attribute.txt

    udevadm info --name=/dev/ttyUSBx --attribute-walk >>>> USB_attribute.txt

    Find {idProduct} and {idVendor}

## Create Rules

    sudo nano 99-bno055.rules
    KERNEL=="ttyUSB*", KERNELS=="1-6.1", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", SYMLINK+="bno055"

#### Activate the rule

    Reload your udevadm rules:

    udevadm control --reload-rules

    sudo udevadm trigger

## Create modprobe bash file for omron

    touch modprobe_omron.sh
