#!/bin/bash
sudo systemctl stop klipper

#make clean KCONFIG_CONFIG=mcu-build-config-skr3
#make -j5 KCONFIG_CONFIG=mcu-build-config-skr3
#./scripts/flash-sdcard.sh /dev/ttyS0 mks-skr3

# RF2000
#make clean KCONFIG_CONFIG=mcu-build-config-rf2000-V6
#make -j5 KCONFIG_CONFIG=mcu-build-config-rf2000-V6
#make flash FLASH_DEVICE=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI046A0Q-if00-port0 KCONFIG_CONFIG=mcu-build-config-rf2000-V6

# Hauptschalter
#make clean KCONFIG_CONFIG=mcu-build-config-power-switch
#make -j5 KCONFIG_CONFIG=mcu-build-config-power-switch
#make flash FLASH_DEVICE=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9QXPRV7-if00-port0 KCONFIG_CONFIG=mcu-build-config-power-switch

# Stromschalter
make clean KCONFIG_CONFIG=mcu-build-config-current-switch
make -j5 KCONFIG_CONFIG=mcu-build-config-current-switch
make flash FLASH_DEVICE=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9PTMHGV-if00-port0 KCONFIG_CONFIG=mcu-build-config-current-switch

# Druckerkuehler
make clean KCONFIG_CONFIG=mcu-build-config-peltier-controller
make -j5 KCONFIG_CONFIG=mcu-build-config-peltier-controller
make flash FLASH_DEVICE=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9QOIMJ6-if00-port0 KCONFIG_CONFIG=mcu-build-config-peltier-controller

sudo systemctl start klipper
