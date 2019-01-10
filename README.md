# usb_example
The Project is based on STM32F407G-DISC1 board, FreeRTOS is used;
The main feature is the ability to change brightness of board LEDs (duty cycle of PWM signals on pins connected to diodes) by Terminal messages to USB Device in VCP mode developped on MCU.

The command consists of 5 bytes:
0 - ID of message (required to be = 1)
1 - duty cycle of Green LED (%)
2 - duty cycle of Orange LED (%)
3 - duty cycle of Red LED (%)
4 - duty cycle of Blue LED (%)
Example (in decimal):
01 20 40 60 80
It means 20%, 40%, 60%, 80% duty cycle for corresponding diodes
If message is correct "OK\r\n" string will be sent back to the terminal

Also when device starts previosly saved settings are downloaded from FLASH, to save current settings it's needed to push User Button.
If saving is done correctly "SAVED!\r\n" string will be sent back to terminal.

Example of Terminal state is showed in ./terminal.png pic in the repository.
