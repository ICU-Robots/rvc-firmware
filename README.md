# rvc-firmware

## Raspberry networking with Raspian

By default the Raspberry Pi registers itself with Bonjour, it should be accessible at raspberrypi.local so one can use:
```sh
ssh pi@raspberrypi.local  # default password is raspberry
```
One can also use `ssh-copy-id` to save the client ssh keys and skip entering a password.


## Arduino code

You might need to install git:
```sh
sudo apt install git
```

Get the code from github:
```sh
git clones https://github.com/ICR-Robots/rvc-firmware.git
```

To compile the firmware and upload to the Arduino board, you can install the *platformio* command line tools.   Instructions can be found at: https://docs.platformio.org/en/latest/core/installation.html.  Short version:
```sh
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/develop/scripts/get-platformio.py)"
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
```

After the one time installation, to activate Python virtualenv created for platformio:
```sh
source ~/.platformio/penv/bin/activate
```
To compile, go in directory that contains platformio.ini (i.e. where Arduino source code is)
```sh
cd ~/rvc/rvc-firmware
platformio run
```
To compile and upload on the Arduino:
```sh
platformio run --target upload
```
