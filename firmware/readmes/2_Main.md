# Main File
## Summary
- This file is the location of the main functions of the firmware. 
- The structure of the main file is as follows:
    - Global variables, objects, 
    - Satellite Commuincaion task
    - Setup Function
    - Loop Function

- There are 3 important functions: 
    - setup()
    - loop()
    - SatCom_iteration()

## Setup Function
- This function handles the setting up of the objects by calling each of their respective setup functions. 
- [Important Note]: 
    - If you want to use the debug print function you must have Serial.begin() at the start of this function, otherwise it is not needed
    - We delay for 5000 milliseconds to give enough time for the serial to properly initialize. This is not needed if you are not concerned with the debug.
    
