#!/bin/bash
echo "SET_DISPLAY_GROUP DISPLAY=\"display control_unit_display\" GROUP=empty_disp" > /home/th86/printer_data/comms/klippy.serial
echo "SET_DISPLAY_GROUP DISPLAY=\"display water_cooler_display\" GROUP=empty_disp" > /home/th86/printer_data/comms/klippy.serial
echo "SET_LED LED=status_control_unit RED=0.05 GREEN=0.04 BLUE=0.1" > /home/th86/printer_data/comms/klippy.serial
sleep 0.2
exit 0
