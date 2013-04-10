freecopter-control
==============
Author: Roberto Marino<br>
Email: formica@ieee.org, roberto.marino@comupter.org

---------------------------

This is the repository containing the firmware of the Control Unit developed for the FreeCopter Project. 
The project was financed from the following universities:<br>
-- University of Messina (Dept. of Physics)<br>
-- University of Genova (DIBRIS)<br>
-- Ecole Centrale de Nantes (IRCCyN)<br> 


The project is based on the ChibiOS RealTime Operating System ~ http://www.chibios.org/

You can download it from the official git repo, in a local directory 'chibios-dir', typing as follow:

git clone https://github.com/mabl/ChibiOS 'chibios-dir'

I personally suggest to download a stable version from  SOURCEFORGE <br>
http://sourceforge.net/projects/chibios/

---------------------------

To compile the IMU firmware against the OS code you need a cross-compiler GNU toolchain.
You can use the script of James Snyder to provide it.

git clone https://github.com/jsnyder/arm-eabi-toolchain arm-cs-tools

Follow his instructions!

---------------------------

SPI
To communicate properly with the IMU Board you need to patch Chibios (2.4.3) with the patch contained in the chibios_patch folder.
<br>
From the console:<br>
cd 'your_chibios_directory'<br>
cd os/hal/platforms/STM32<br>
patch -p1 < chibios_spi_slave.patch

