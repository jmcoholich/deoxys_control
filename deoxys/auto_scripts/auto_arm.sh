#!/bin/bash
. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

DEBUG_MODE="off"
ENABLE_COREDUMP=1

while [[ "$1" == --* ]]; do
    case "$1" in
        --gdb)
            DEBUG_MODE="gdb"
            shift
            ;;
        --gdb-bt)
            DEBUG_MODE="gdb-bt"
            shift
            ;;
        --no-coredump)
            ENABLE_COREDUMP=0
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [ "$ENABLE_COREDUMP" -eq 1 ]; then
    ulimit -c unlimited 2>/dev/null
fi

printf "${BIRed} Make sure you are in the Performance Mode!!! ${Color_Off} \n"

RTOS_MODE=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

echo $RTOS_MODE

for mode in $RTOS_MODE
do
    if [ "${mode}" = "powersave" ]; then
	printf "${BIRed} Not in Performance Mode, will cause errors for franka codebase!!! ${Color_Off} \n"
    fi
done

run_franka_interface() {
    if [ "$DEBUG_MODE" = "gdb" ]; then
        gdb --args bin/franka-interface "$@"
    elif [ "$DEBUG_MODE" = "gdb-bt" ]; then
        gdb -q -ex run -ex "bt" -ex "thread apply all bt" -ex quit --args bin/franka-interface "$@"
    else
        bin/franka-interface "$@"
    fi
}

while true
do
    run_franka_interface "$@"
    status=$?
    if [ $status -ne 0 ]; then
        echo "franka-interface exited with code $status; stopping auto-restart."
        if [ "$ENABLE_COREDUMP" -eq 1 ]; then
            echo "Core dumps enabled. If a core file exists, inspect with: gdb bin/franka-interface core"
            echo "If no core file appears, try: coredumpctl info bin/franka-interface"
        fi
        exit $status
    fi
    sleep 1
done
