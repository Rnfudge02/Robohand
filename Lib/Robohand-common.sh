#file Robohand-common.sh
#brief Robotic hand control script utilities
#details Used for other robohand modules, which each handle different methods of connection
#author Robert Fudge <rnfudge@mun.ca>
#date 2025

#ASCII escape formatting sequences
RESET="\033[0m"
BOLD="\033[1m"
DIM="\033[2m"
ITALIC="\033[3m"
UNDERLINE="\033[4m"
BLINK="\033[5m"

#ASCII foreground formatting sequences
FG_BLACK="\033[30m"
FG_RED="\033[31m"
FG_GREEN="\033[32m"
FG_YELLOW="\033[33m"
FG_BLUE="\033[34m"
FG_MAGENTA="\033[35m"
FG_CYAN="\033[36m"
FG_WHITE="\033[37m"

#ASCII background formatting sequences
BG_BLACK="\033[40m"
BG_RED="\033[41m"
BG_GREEN="\033[42m"
BG_YELLOW="\033[43m"
BG_BLUE="\033[44m"
BG_MAGENTA="\033[45m"
BG_CYAN="\033[46m"
BG_WHITE="\033[47m"

#Function to get the real path to the device - Used to get the device path and determine
#whether it is a pico or pico w, and whether its running or in flash mode
get_dev_path() {
    echo -e "${FG_CYAN}[Container Controller]${FG_BLUE} retrieving /dev mapping for $1${RESET}" 
    ID_VEND=${1%:*}
    ID_PROD=${1#*:}
    for path in `find /sys/ -name idVendor 2>/dev/null | rev | cut -d/ -f 2- | rev`; do
        if grep -q $ID_VEND $path/idVendor; then
            if grep -q $ID_PROD $path/idProduct; then
                find $path -name 'device' | rev | cut -d / -f 2 | rev
            fi
        fi
    done
}

#Function to set/update variable
update_var() {
    var_name="$1"
    new_value="$2"
    
    if grep -qE "^export ${var_name}=" ~/.bashrc; then
        sed -i "s|^export ${var_name}=.*|export ${var_name}=${new_value}|" ~/.bashrc
    else
        echo "export ${var_name}=${new_value}" >> ~/.bashrc
    fi
}