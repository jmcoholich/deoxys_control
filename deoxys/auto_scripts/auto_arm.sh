#!/bin/bash
. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

printf "${BIRed} Make sure you are in the Performance Mode!!! ${Color_Off} \n"

RTOS_MODE=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

echo $RTOS_MODE

for mode in $RTOS_MODE
do
    if [ "${mode}" = "powersave" ]; then
	printf "${BIRed} Not in Performance Mode, will cause errors for franka codebase!!! ${Color_Off} \n"
    fi
done

EVAL_MODE=0
FRANKA_ARGS=()

for arg in "$@"
do
    if [ "$arg" = "--eval" ]; then
        EVAL_MODE=1
    else
        FRANKA_ARGS+=("$arg")
    fi
done

while true
do

    bin/franka-interface "${FRANKA_ARGS[@]}"

    if [ "$EVAL_MODE" -eq 0 ]; then
        printf "${BICyan}franka-interface exited. Press any key to resume...${Color_Off}\n"
        read -r -n 1 -s
        printf "\n"
    fi

    sleep 0.1

done
