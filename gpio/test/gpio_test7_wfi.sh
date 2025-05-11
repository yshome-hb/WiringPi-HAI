#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # Reset color 

UNITTEST_OK=true
TIMEOUT=8
iteration=0
GPIOIN=26
GPIOOUT=19

wait_with_timeout() {
    local pid=$1
    local timeout=$2
    iteration=0

    local toggle=0
    while kill -0 "$pid" 2>/dev/null; do
        if [ $iteration -ge $timeout ]; then
            kill -9 "$pid" 2>/dev/null
            echo -e "${RED}ERROR: timeout ${timeout}s reached ${NC}"
            return -1
        fi
        echo -n .
        sleep 1
        toggle=$((1 - toggle))
        gpio -g write $GPIOOUT $toggle
        echo -n ${toggle}
        iteration=$((iteration + 1))
    done

    wait "$pid"
    return $?
}

wfi_test() {
    local edge=$1
    local set_iter=$2
    local set_iterOK=$3

    #wfi test
    if [ $iteration -eq 0 ]; then
        gpio -g wfi $GPIOIN "$edge" &
    else
        gpio -g wfi $GPIOIN "$edge" "$set_iter" &
    fi
    GPIO_PID=$!
    wait_with_timeout "$GPIO_PID" "$TIMEOUT"
    EXIT_CODE=$?
    echo 
    if [ $EXIT_CODE -eq 0 ]; then
        if [ "$set_iter" -gt 0 ]; then
            if [ "$iteration" -eq "$set_iterOK" ]; then
                echo -e "${GREEN}wfi "$edge" $set_iter iteration passt (exit code 0)${NC}"
            else
                echo -e "${RED}wfi "$edge" failed, $iteration iterations of $set_iter is wrong${NC}"
                UNITTEST_OK=false
            fi
        else
            echo -e "${GREEN}wfi "$edge" passt (exit code 0)${NC}"
        fi
    else
        echo -e "${RED}wfi "$edge" failed (exit code $EXIT_CODE)${NC}"
        UNITTEST_OK=false
    fi
}

echo 
echo Unit test gpio GPIO${GPIOOUT} and GPIO${GPIOIN} - functions: mode, write, wfi
echo ------------------------------------------------------------------
echo 

#prepare trigger out
gpio -g mode $GPIOOUT out

gpio -g write $GPIOOUT 0
wfi_test "rising" 0 0

gpio -g write $GPIOOUT 1
wfi_test "falling" 0 0

#wfi iteration test

gpio -g write $GPIOOUT 0
wfi_test "rising" 4 7

gpio -g write $GPIOOUT 1
wfi_test "falling" 4 8

gpio -g write $GPIOOUT 0
wfi_test "both" 4 4

gpio -g write $GPIOOUT 0
gpio -g mode $GPIOOUT in

### #wfi timeout test
gpio -g wfi $GPIOIN rising 2 4 &
GPIO_PID=$!
wait_with_timeout "$GPIO_PID" "$TIMEOUT"
EXIT_CODE=$?
echo 
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}wfi timeout passed (exit code 0)${NC}"
else
    echo -e "${RED}wfi timeout failed (code $EXIT_CODE)${NC}"
    UNITTEST_OK=false
fi


if [ ${UNITTEST_OK} = true ]; then
    echo -e "\n\n${GREEN}Unit test result OK.${NC}"
    exit 0
else
    echo -e "\n\n${RED}Unit test result failed.${NC}"
    exit 1
fi