FreeRTOS v11.0.1 + LVGL v9 porting steps
===

the repository for show how to porting FreeRTOS and LVGL v9 on STM32F4xx

Hardware
===
* [NUCLEO-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)
* [LCD Module - 2.4" ILI9341 SPI Interface](http://www.lcdwiki.com/2.4inch_SPI_Module_ILI9341_SKU:MSP2402)


Signal wire
=========

use CN7 of NUCLEO-F429ZI to connection the SPI LCD


 SPI data signal of LCD
 -----
     LCD Module            NUCLEO-F429ZI
     SDI(MOSI)   <->         PA7          // SPI Data Input/SPI Master Output Slave Input
     SDO(MISO)   <->         PA6          // SPI Data Output/SPI Master Input Slave Output, option connection
 LCD control signal
 ----
     LCD Module            NUCLEO-F429ZI
     LED         <->         PB9          // Backlight control
     SCK         <->         PA5          // clock of SPI
     DC/RS       <->         PD14         // Data/Control indicator signal
     RST         <->         PD15         // Reset of LCD
     CS          <->         PF12         // chip select of SPI
