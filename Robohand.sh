#!/bin/bash

#Copyright [2025] [Robert Fudge]
#SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

#Robohand Container Controller

source ./Lib/Robohand-common.sh

#Get project root directory (where script is executed)
PROJECT_ROOT=$(pwd)
NEW_SDK_PATH="$PROJECT_ROOT/Dependencies/pico-sdk"

#Backup .bashrc
cp ~/.bashrc ~/.bashrc.bak

#Define other variables
NEW_PLATFORM="rp2040"
NEW_BOARD="pico"

#Parse command line arguments
while getopts "abdhip:s:" options; do
    case ${options} in
        #AI-prompt - generate a template prompt document
        a)
            echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Generating prompt template file"
            rm -rf ./prompt_file.txt

            #Core system information
            echo -e "I am trying to develop a robotic hand using C, the Raspberry Pi Pico SDK, and \n\
the MicroROS library. I am trying to keep the code lightweight, extensible, and multithreaded. \n\
The system hardware is structured as follows: \n\
- 5 MG996R Servos controlled via PWM on GPIO 11-15 \n\
- ADS1115 connected to I2C0 Pins 0 & 1 \n\
- BME280 connected to I2C0 Pins 0 & 1 \n\
- MPU6050 connected to I2C0 Pins 0 & 1 \n\
- QMC5883L connected to I2C0 Pins 0 & 1 \n\
\n\
The system code is structured as follows: Robohand_advanced.c/h contains the \n\
system hardware implementation, allowing for interaction with the physical hardware \n\
via the Pico SDK, and allows for a variety of backends, callbacks, interrupts, or DMA. \n\
Robohand_usb.c contains code to provide a minimal terminal experience \n\
to a client using the Pico SDK, and allow for reading and interacting with the sensors \n\
using the accessible functions contained in Robohand.h. Robohand_uros.c/h is responsible \n\
for allowing host devices running the Micro-ROS client to interact with the hand via the \n\
interface provided by Robohand.c/h. \n\
" >> prompt_file.txt

            #Append file information to end of prompt
            echo -e "The current impelmentation is shown below. Robohand.h header file" >> prompt_file.txt

            cat ./Include/Robohand.h >> prompt_file.txt

            echo -e "Robohand.h header file end, begin Robohand.c source file"  >> prompt_file.txt

            cat ./Src/Robohand.c >> prompt_file.txt

            echo -e "Robohand.c header file end, begin Robohand_advanced.h" >> prompt_file.txt

            cat ./Include/Robohand_advanced.h >> prompt_file.txt

            echo -e "End Robohand_advanced.h, begin Robohand_advanced.c" >> prompt_file.txt

            cat ./Src/Robohand_advanced.c >> prompt_file.txt
            
            echo -e "End Robohand_advanced.c, begin CMakelists.txt" >> prompt_file.txt

            cat CMakeLists.txt >> prompt_file.txt

            #Insert user request
            echo -e $2 >> prompt_file.txt

            echo -e "Please try not to overcomplicate suggestions. Do not hallucinate Pi Pico SDK \n\
functions, or MicroROS functions. Consider and provide information regarding the speed versus \n\
complexity of the problem. Double check all suggestions provided to CMakeLists.txt" >> prompt_file.txt
        ;;

        #Build - controls building of target container
        b)
            echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Building Robohand Projects.${RESET}"

            #Update or add variables
            update_var PICO_SDK_PATH "$NEW_SDK_PATH"
            update_var PICO_PLATFORM "$NEW_PLATFORM"
            update_var PICO_BOARD "$NEW_BOARD"

            mkdir -p Build
            rm -rf ./Build/*
            cd ./Build/

            cmake -DPICO_BOARD=pico -DUSE_DMA=OFF ..
            make Robohand_usb Robohand_uros -j$(nproc)

            cd ..
            mkdir -p ./Outputs
            mv ./Build/Robohand_uros.uf2 ./Outputs/Robohand_uros.uf2
            mv ./Build/Robohand_usb.uf2 ./Outputs/Robohand_usb.uf2

            rm -rf ./Build/*
            cd ./Build/

            echo -e "${FG_GREEN}[Robohand Controller]${FG_GREEN} Robohand build script finished.${RESET}"
        ;;

        d)
            echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Generating doxygen documentation.${RESET}"

            mkdir -p ./Documentation

            if [ ! -f ./Robohand.doxyfile ]; then
                doxygen -g Robohand.doxyfile
            fi

            doxygen Robohand.doxyfile

        ;;

        #Help - Displays the valid commands for the controller
        h)
            echo -e "${FG_GREEN}${BOLD}Robohand Controller V1.0 - Developed by Robert Fudge${RESET}"
            echo -e "${FG_GREEN}Valid commands are listed below:"

            echo -e "ARGUMENT       NAME            INFO"
            echo -e "-a             AI-prompt       Formats a prompt template for passing into LLM's."
            echo -e "-b             Build           Build robohand projects."
            echo -e "-d             Document        Generates local copy of Doxygen documentation for viewing purposes."
            echo -e "-i             Init            Initialize host software packages to well-known environment, needs sudo."
            echo -e "-p             push            Pushes desired project to the device if it is in flash mode."
            echo -e "-s             Start           Start desired container, pass in suffix of top-level dockerfile.${RESET}"
        ;;

        #Init - build for opposite architecture as target - NOT WORKING, issue with transferring build stages
        i)
            echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Initializing system for Robohand communication.${RESET}"

            #Install needed apt packages
            sudo apt update && sudo apt install -y minicom cmake build-essential gcc g++ gcc-arm-none-eabi doxygen libxapian30 graphviz

            #Initialize submodules
            git submodule update --init --recursive

            #Build micro-ROS library first
            cd Dependencies/micro_ros_pico_sdk
            mkdir build && cd build
            cmake ..
            make -j$(nproc)

            echo -e "${FG_GREEN}[Robohand Controller]${FG_GREEN} Robohand system initialized.${RESET}"

        ;;

        #Start - Start the environment to communicate with the desired project
        s)
            if [[ "$2" == "uros" ]]; then
                echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Starting host code for ${1}.${RESET}"
                
                docker pull microros/micro-ros-agent:humble
                docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
                
                echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Closing terminal instance.${RESET}"

            elif [[ "$2" == "usb" ]]; then
                echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Starting host code for ${2}.${RESET}"
                
                sudo minicom -b 115200 -o -D /dev/ttyACM0
                
                echo -e "${FG_GREEN}[Robohand Controller]${FG_BLUE} Closing terminal instance.${RESET}"

            else
                echo -e "${FG_GREEN}[Robohand Controller]${FG_RED} Error: Invalid project selected.${RESET}"
                exit 4
            fi

        ;;
    esac
done