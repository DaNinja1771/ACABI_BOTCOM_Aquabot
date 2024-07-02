# Installation
## Description
- This document contains the steps needed to install the firmware on a desired **BOTCOM Year 2** drone but does **NOT** describe how to install the software tools used to do so. This is to avoid incorrect information if the software provider changes installation procedures. 
- The procedure starts at creating a project in platform.io to house the firmware source code and is correct as of **4/1/2024**
## Tools
- Software tools:
    - Arduino IDE
        - This will not be used directly but used by platform.io when installed
    - VS Code
    - Platform.io
    - Python
        - Use whatever version of python is needed by platform.io (as of now 5/1/2024 it is Python3)
- Hardware tools:
    - Laptop capable of running Python3
    - Cable to connect your laptop to the drone. 
        - Needing a (port of your choice) to Micro USB type B.
            - Make sure the cable has the data pin as many are made without it
    - The fully assembled Botcom Year 2 drone
## Installation
### Arduino IDE
This can be installed by searching for the arduino ide in your web browser and following the download instructions based on your operating system
### VS Code
This can be installed by searching for VS code in your web browser and following the download instructions based on your operating system
### platform.io
This can be installed inside of VS Code once downloaded. In the extensions tab you can search for platform.io and follow their installation instructions. You will need Python 3.8 or later and need to be able to find the python.exe on your system if platform.io cannot find it on its own.
### Python 3
This can be installed by searching for Python 3 in your web browser and following the installation instructions on their web page.

## Accessing the Project in Platform.io

Once the above software has been installed on your system you can begin with the creation or importation of the files. Creating and importing will be gone over in the next sections as a list of steps.
### Creating a Project
1. Open VS Code
2. Start platform.io by accessing the platform.io tab on the left side of the window.
3. Once on the home screen for platform.io you can either use their default quick links or use the tab on the left labeled projects and selecting create new project.
4. In the project wizard there will be 3 input fields: 
    - [Project Name]: **Your Choice**
    - [Board]       : **Espressif ESP32 Dev Module**
    - [Frame Work]  : **Arduino**
5. When creating the first project it will take some time to make and will be faster after
6. When the project has been made you can add all of the files found in the github to the project folder. 
    

