#!/bin/bash
EXEC=$1

if [ "${EXEC}" == "" ]; then
	EXEC=build/StalkerNG.hex
fi
if [ ! -f ${EXEC} ]; then
	echo "Build Application first. Please run 'make clean' & 'make' !"
	exit 1
fi
openocd -f openocd/stm32f4eval.cfg \
-c "init; targets; reset init; wait_halt; poll; flash write_image erase unlock ${EXEC}; reset run; shutdown"
