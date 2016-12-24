# STM32 F7 Discovery
Experiments with a STM32F7 Discovery kit. This kit is known as DISCO-F746NG and is is described at this [site](http://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-discovery-kits/32f746gdiscovery.html "site").

## System Workbench    
The System Workbench toolchain, is a free software development environment based on Eclipse, which supports the full range of STM32 microcontrollers and associated boards. System Workbench is planned to be used for this project.

## STM32CubeMX
STM32CubeMX which is a graphical software configuration tool that can generate initialization code. The base for new projects will be done by aid of this tool.  

## Motor Shield Control
The Discovery kit is equipped with Arduino compatible connectors. The plan is the connect a [Motor Shield](https://www.arduino.cc/en/Main/ArduinoMotorShieldR3 "Motor Shield") to be able to run a pair of motors.  

## Notes  
Had to change to Internal builder as Builder type to get a Nucleo based test project from STM32 Mx cube to compile.   
