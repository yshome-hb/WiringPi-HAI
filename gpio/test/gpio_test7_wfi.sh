#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # Reset color 

UNITTEST_OK=true
TIMEOUT=8
iteration=0

wait_with_timeout() {
    local pid=$1
    local timeout=$2
    iteration=0

    while kill -0 "$pid" 2>/dev/null; do
        if [ $iteration -ge $timeout ]; then
            kill -9 "$pid" 2>/dev/null
            echo -e "${RED}ERROR: timeout ${timeout}s reached ${NC}"
            return -1
        fi
        gpio -g write 19  1
        sleep 1
        echo -n .
        gpio -g write 19  0
        iteration=$((iteration + 1))
    done

    wait "$pid"
    return $?
}

echo 
echo Unit test gpio GPIO19 and GPIO26 - functions: mode, write, wfi
echo ------------------------------------------------------------------
echo 

#prepare trigger out
gpio -g mode 19 out
gpio -g write 19 0

#wfi test
gpio -g wfi 26 rising &
GPIO_PID=$!
wait_with_timeout "$GPIO_PID" "$TIMEOUT"
EXIT_CODE=$?
echo 
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}wfi passt (exit code 0)${NC}"
else
    echo -e "${RED}wfi failed (exit code $EXIT_CODE)${NC}"
    UNITTEST_OK=false
fi

#wfi iteration test
gpio -g wfi 26 rising 4 &
GPIO_PID=$!
wait_with_timeout "$GPIO_PID" "$TIMEOUT"
EXIT_CODE=$?
echo 
if [ $EXIT_CODE -eq 0 ]; then
    if [ "$iteration" -eq 5 ]; then
      echo -e "${GREEN}wfi iteration passt (exit code 0)${NC}"
    else
      echo -e "${RED}wfi failed, iteration $iteration wrong${NC}"
      UNITTEST_OK=false
    fi
else
    echo -e "${RED}wfi failed (exit code $EXIT_CODE)${NC}"
    UNITTEST_OK=false
fi

gpio -g write 19 0 
gpio -g mode 19 in

### #wfi timeout test
gpio -g wfi 26 rising 2 4 &
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