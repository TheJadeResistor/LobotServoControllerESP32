myController.getRealPosition(ServoID);

ESP32/Arduino control with real-time position fetching. The original library didn't compile for the ESP32 Dev Module due to some variable definitions getting messed up with ESP32 selected, fixed here. Also added the ability to fetch a servo angle/position at any time in real-time, meaning you can retrieve real servo positions at any time and after reboot.

This project uses the Hiwonder 'Serial Bus Servo Controller: Hiwonder Serial Bus Servo Controller Communication Tester' and LX-224 serial bus servos.

Real-time position fecthing was assisted by the Bus Servo Controller Communication Protocol manual (https://www.hiwonder.com.cn/store/learn?key=servo).

Each command for this serial bus controller consisted of a specific sequence of bytes to send. When you call 'yourServoController.getRealPosition(int servoNum)' for a servo it sends the sequence and a different sequence is returned. buf[7] and buf[6] are the return packages from calling the command to retrieve position data for a given servo. The return packages are single byte hex digits up to 0xFF or 255. From the manual example I knew that the packages returning 0xF4 and 0x01 resulted in the decimal digit of 500, being position 500 of the servo. Deciphering this shows that it is in little endian format and this equation works: MSB*256+LSB=decimal_value. The most significant bit (MSB) is the last bit as it is in little endian format (the least significant byte (LSB) is stored at the lowest memory address, and the most significant byte (MSB) is stored at the highest memory address) so that equation works as MSB=0x01 and LSB=0xF4 so 0x01*256+0xF4 = 1*256+244 = 500. 

This function converts hex to decimal so the calculation can be performed: #define HEX_TO_DEC(A) ((((A) & 0xF0) >> 4) * 16 + ((A) & 0x0F))


Making this work for the ESP32 consisted of changing like one line to change a bit of the logic to work the same but format differently.
