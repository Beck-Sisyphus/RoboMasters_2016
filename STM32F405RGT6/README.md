# RoboMasters_2016_Open_Source
The open source code running on the trapezoid board for STM32F405RGT6. Thanks for DJI.

## Hardware Configuration
	(1) Hold trapezoid board with smaller edge facing right
	(2) From top to bottom, the programming connector pins are
		1: 3.3V
		2: GND
		3: SWCLK
		4: SWDIO
	(3) Connect to ST-Link using the following wire colors
		SWCLK -> BROWN
		SWDIO -> WHITE
		GND -> BLACK
		3.3V -> RED

## Setup
	(1) Download Keil MDK-ARM from https://www.keil.com/demo/eval/arm.htm and install.
		(Must provide contact details, I just put UW for the company and address)
	(2) Download and install the MDK v4 Legacy Support Cortex-M package from http://www2.keil.com/mdk5/legacy/
	(3) Fetch the repository.
		$ git clone https://github.com/Beck-Sisyphus/RoboMasters_2016_Open_Source.git
		or download the zip from https://github.com/Beck-Sisyphus/RoboMasters_2016_Open_Source/archive/master.zip
	(4) In /STM32F405RGT6/Project, double click on STM32F4.uvprojx to open the project with Keil

## Change Project Settings
	(1) Flash -> Configure Flash Tools...
	(2) Under "Use Target Driver...", select ST-Link Debugger
	(3) Next to the selection box, click Settings
	(4) Select Erase Full Chip, and leave everything at default values

## Program the Board
	(1) Project -> Build Target
	(2) Flash -> Download
