# BART-FSW

Embedded C code for the BART sounding rocket control board. This repository is functionally identical to the GTXR flight software repository. This repo uses Lazo Attar's configuration manager toolchain to manage hardware configurations and allow for future portability to other STM32 platforms. 

## Hardware Description
### Sensors
### Actuators

## First-Time Setup
To run this toolchain, you will need Docker Desktop **(insert link here)**

**(Insert Docker instructions here)**

To run this toolchain on Linux without Docker, you will need the following:
- STM32 CubeMX for generating HAL libraries (other than the ones already contained in the repository)
- Make for compiling the project
- OpenOCD for programming and debugging

## Building the Project
This section describes how to configure, build, and deploy the project. Lazo has created a very useful set of tools for configuring this repository that make this entire process significantly easier. The general steps are: 

1. Configure the Makefile
2. Build the project
3. Flash the program

These steps are elaborated in the sections below.

### Makefile Configuration
The makefile configuration tool sets up the makefile given the configuration/processor we want to use, and can be called via: 

```./tools/configure.sh -c data/configs/main -p . -m stm32h7 -e fatfs -e dsp```. 

The flags for this tool are shown below:

- ```-c``` path to the CubeMX config we want to use. In the example, we are using the config located in ```data/configs/main```. To compile for other MCUs, we simply create a new 
- ```-p``` is the type of processor we want to compile to. For the example above, we are compiling for the STM32H7, which is the processor on the ADCS board. 
- ```-e``` is a flag we use to add extra libraries to the makefile. In the example, we are adding FATFS and CMSIS-DSP. These should be configured to be added to the repository through CubeMX. 

After configuring the makefile, we need to fix it to properly incorporate CMSIS-DSP (this should be accounted for in our initial config script at some point). This can be called via: 

```./tools/fix_make.sh```

### Building the project
Once the makefile is configured, we can build the project. We can do this simply through make, with a call to: 

```make -j$(nproc)```

The ```-j$(nproc)``` flag uses all your computer's available cores so that the code is built faster by using parallel processing.

### Flashing code
Once the program is built, we can flash the board using OpenOCD. We do this with the command: 

```openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program ./build/main/main.bin 0x08000000 verify exit reset"```