<div id="readme" class="Box-body readme blob js-code-block-container">
  <article class="markdown-body entry-content p-3 p-md-6" itemprop="text"><p><a href="https://www.microchip.com" rel="nofollow"><img src="https://camo.githubusercontent.com/5fb5505f69a28ff407841612dfe2b7004f210594/68747470733a2f2f636c6475702e636f6d2f553071684c7742696a462e706e67" alt="MCHP" data-canonical-src="https://cldup.com/U0qhLwBijF.png" style="max-width:100%;"></a></p>

# KSZ8851SNL

The repository provides the driver for enabling the TCP and UDP communication in KSZ8851SNL ethernet controller. The user can build upon this application to extend the capabilities of the KSZ8851SNL controller.

## Required Tools:

Hardware tools:

* KSZ8851SNL Eval Board
* PIC18F47K42 (40-pin, PDIP) MCU
* The Curiosity development board
* Ethernet cables (RJ45)
* Ethernet switch with power supply

Software tools:

* MPLAB® X IDE 5.25 or later
* MPLAB® Code Configurator (Plugin)
* XC8® Compiler 2.05 or later
* Microchip TCP/IP Lite Stack (AN1921)
* Wireshark Packet Analyzer

## MCC Settings
Here are the settings used for MSSP, MAC, TMR, Pin module, TCP/IP Lite and System module.

### System Module Settings
![System Module Settings](image/SystemModule.PNG)

### System Conguration Register Settings
![System Configuration Register Settings](image/SystemConfigRegister.PNG)

### TMR Settings
![TMR Settings](image/TMR1.PNG)

### MSSP Settings
![MSSP Settings](image/SPI.PNG)

### MAC Settings
![MAC Settings](image/MAC.PNG)

### TCP/IP Settings
![TCP/IP Settings](image/TCPIP.PNG)

### Pin Module Settings
![Pin Module Settings](image/PinModule.PNG)

### Pin Manager Settings
![Pin Manager Settings](image/PinManager.PNG)