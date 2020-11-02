# ESP8266

I recommended lubuntu 18.4 and python3 for this tutorial

# First step: Download ESP8266 RTOS SDK
```
cd ~/esp
git clone --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git
```

# Second step: Install
```
cd ~/esp/ESP8266_RTOS_SDK
./install.sh
```

# Export tools
`. ./export.sh`

now you can use make tools with xtensa_106lx.
Create an alias for make tools 

`sudo gedit ~/.bashrc`

Write this command start of script.

`alias idf='. /home/ahmet/esp/ESP8266_RTOS_SDK/export.sh''

There two way to compile and flash projects.(recommended 2. way)

after that we can start to use eclipse as a compiler.But before it we must update java 8 to java 11 with this command.

`sudo apt install default-java`

Download eclipse from this link: https://www.eclipse.org/downloads/ 

Install eclipse C/C++ developer kit

this tutorial show how to configs and other stuffs : https://github.com/espressif/idf-eclipse-plugin/blob/master/README.md

Note: When we changed projects compiler step in a bug like : Build tools not configured correctly 


# Second way (Without compiler)

After export tools we can compile and flash our first program
```
cd ~/esp
cp -r /ESP8266_RTOS_SDK/examples/get-started/blink
cd blink
```
Write this command to export tools

`idf`


Regulate config.(Default configs enough yet.)

`make menuconfig`
# Learn port of device.

`ls /dev/tty*`
run make menuconfig  Serial flasher config > Default serial port <your port>

# Build the project.

`make`

# Flash 

`make flash`


# Eror of permission :

`sudo chmod 777 <your port>`
try again flashing
        
# congratulations

I wait your questions

Regards

´´´
