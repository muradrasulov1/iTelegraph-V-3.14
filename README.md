### Project Description

The project is a morse code - character bidirectional translator. It uses the red LED on the KL board to display a morse code translation of a string of characters typed using a USB keyboard and uses SW1 and SW3 to indicate inputted dots and dashes respectively. For morse to character translation, the desired sequence is inputted one letter (digit) at a time using the SW1 and SW2, dot and dash, respectively. 

To indicate whether the device is translating morse code to characters or characters to morse code, the green LED on the KL board is off or on respectively in either case. The device also uses the UART protocol to display the current mode, characters typed from the USB keyboard and characters translated from morse code. This technically makes the device interfaceable with any bash terminal that has a usb to serial driver installed, however the MCUExpresso debug console is recommended for simplicity.

### Technical

The system is always initialized to the Morse to Character mode. In this mode, the Dots and Dashes are entered using the SW1 and SW3 buttons that come with the board. Every time a character is completed, the Enter key is pressed on the keyboard, which, in case a valid character was entered in morse, will have the sequence translated to Character, instantly showing it in the Debug Console. Once the word is completed, the Space bar is pressed to allow for a delimiter in the Console output.

Most of the code was created and the existing functions were modified in app.c and host_keyboard.c. The first one acted as THE essential main function, where all the action happened, including the peripherals/LEDs/interrupts initializations. The latter function played a key role in the setting up of the keyboard peripheral, as it allowed to have direct actions done to the keyboard's process buffer. This is where I defined functions to parse data packets coming from the keyboard.

For the implementation of the translator, I used the binary nature of morse code to implement a binary tree that stores all of the characters as their keyboard scan codes, which are then converted to characters using a print function in the usb library. I also used an array to store the morse translation of each character in binary using unsigned integer values.

#### In Sources Folder: 
host_keyboard.c -> Backbone of Keyboard Operation | read_key (implements bin. tree & unsigned array) & USB_HostKeyboardProcessBuffer (facilitates keyboard to microcontroller data transfer) == main tandem

app.c -> Interrupts' Definitions (for touch, push buttons); Hardware Initialization; MAIN function (While loop mainly responsible for M->C | C->M is done via a single keyboard interrupt once a full message is fed into the buffer | fed message shown on via 2 LEDs all togethet); M2C Binary Tree
In app.c: SW1, SW2 -> simple push buttons | TSI -> capacitive touch sensor


### USB

The communication protocol used for gathering data from the keyboard is called the USB protocol. This is a protocol that uses a D+ and D- signal to send data packets using an encoding scheme called NRZI, which interprets a change in state of the data pins as a 0 and the state remaining the same as a 1.

USB protocol uses NRZI (non-return to zero) encoding to communicate information about the speed of the device and the contents of the data being sent from the keyboard (i.e. device address, keys pressed, etc.). Luckily, the KL boards SDK provides a library for reading from the buffer description table that is constantly updated by the board when the clock is enabled to the USB port, making the process of reading keys as simple as checking for when changes are made.

### Mode Switching

For mode switching, I also used the capacitive touch sensor that is built into the board. An interrupt handler was used to determine when the touch sensor is being touched as used this information to toggle the mode between character to morse mode and morse to character mode. One of the challenges of using the interrupt handler to toggle modes is that a single tap of the sensor can initiate multiple interrupts, thus potentially leading to the unintended effect of toggling modes more times than necessary. However, to fix this problem I simply used an integer variable called “touch” that is either 0 or 1 and used the interrupt handler to set the variable to 1. A main while loop was later used to debounce the touch sensor. A similar method was used for debouncing the dot/dash buttons, as receiving an incorrect number of presses can have a dramatic effect on message translation.

To switch to the C->M mode, the touch sensor is pressed once, interrupting all the current processes and switching the mode. In this mode, the user has to input the desired sequence from the keyboard, followed by the Enter button, which ends the input and sends the characters to the board to translate. To display the output, the Red LED is used. Every single character is separated with a delay to allow for readability, and every word is separated with twice the delay for the same reason. Once output is shown, user can enter a new sequence of words/letters to translate it to Morse. To switch modes, the same one touch of the TCM sensor is needed.
