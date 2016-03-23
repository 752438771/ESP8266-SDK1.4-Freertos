#!/bin/bash

echo "gen_misc.sh version 20151112"
echo "gen flash=4Mbytes"
#touch user/user_main.c

export SDK_PATH=~/ESP8266_RTOS_SDK-master
export BIN_PATH=~/ESP8266_RTOS_SDK-master/bin

boot=new
app=1
spi_speed=2
spi_mode=0
spi_size_map=4

make COMPILE=gcc BOOT=$boot APP=$app SPI_SPEED=$spi_speed SPI_MODE=$spi_mode SPI_SIZE_MAP=$spi_size_map