# ttyit
i2c terminal driver

# DKMS

 su -

 apt install build-essential apt install linux-headers 

 apt install dkms 

 cd /usr/src

 git clone ttyit

 cd ttyit

 dkms add -m ttyit -v 1.0

 dkms build -m ttyit -v 1.0

 dkms install -m ttyit -v 1.0


 echo ttyit 0x55 > /sys/bus/i2c/devices/i2c-0/new_device
 
