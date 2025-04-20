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

            cat ./README.md >> prompt_file.txt

            
            #Show tree representation
            echo -e "The projects tree representation is given below.\n" >> prompt_file.txt
            tree ./Include ./Src >> prompt_file.txt

            #Insert user request
            echo -e $2 >> prompt_file.txt
            
            #Add project file context
            for FILE in ./Include/*; do cat $FILE >> prompt_file.txt; done

            for FILE in ./Src/Core/*; do cat $FILE >> prompt_file.txt; done

            for FILE in ./Src/Robohand_*; do cat $FILE >> prompt_file.txt; done

            cat CMakeLists.txt >> prompt_file.txt

            #Additional instructions
            echo -e "Please try not to overcomplicate suggestions. Do not hallucinate Pi Pico SDK \n\
functions, or MicroROS functions. Consider and provide information regarding the speed of solutions. \n\
Double check all suggestions provided to CMakeLists.txt" >> prompt_file.txt
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