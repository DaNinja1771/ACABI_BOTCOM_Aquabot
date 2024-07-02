import os
import sys
import serial
import threading
import json
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QGroupBox, QSizePolicy, QHBoxLayout
from PyQt5.QtCore import pyqtSignal, QObject, QCoreApplication
import re
from datetime import datetime, timezone

script_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_directory)

json_file_path = "params.json"
path_option = 3

def get_params():
    try:
        with open(json_file_path, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"The file {json_file_path} was not found.")
        return None
    except json.JSONDecodeError:
        print("Error decoding JSON from the file.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

class SerialThread(QObject):
    newData = pyqtSignal(str)

    def __init__(self, port, baud, file_path):
        super().__init__()
        self.serial = serial.Serial(port, baudrate=baud, timeout=1)
        self.outfile = open(file_path, 'a')

    def run(self):
        while True:
            try:
                line = self.serial.readline()
                if line:
                    decoded_line = line.decode('utf-8').strip()
                    self.newData.emit(decoded_line)  # Emit the signal with the read data
                    self.outfile.write(decoded_line + '\n')
                    self.outfile.flush()
            except Exception as e:
                print(f"Error: {e}")
                break

class DroneDebuggerGUI(QWidget):
    def __init__(self, port, baud, file_path):
        super().__init__()
        self.initUI()
        self.serialThread = SerialThread(port, baud, file_path)
        self.serialThread.newData.connect(self.updateData)
        self.thread = threading.Thread(target=self.serialThread.run)
        self.thread.start()
        
    def closeEvent(self, event):
        # Perform any clean-up tasks here
        print("Closing application...")
        if self.thread.is_alive():
            # Optionally send a signal to stop the thread or use a flag
            self.serialThread.terminate()  # You might need to define a terminate method in SerialThread
            self.thread.join()  # Wait for the thread to finish

        # Safely close the application
        QCoreApplication.quit()

    def initUI(self):
        self.masterLayout = QHBoxLayout()  # This is the top-level layout
        self.leftLayout = QVBoxLayout()     # For all modules except Network Module
        self.rightLayout = QVBoxLayout()    # Specifically for Network Module

        self.setLayout(self.masterLayout)
        self.setWindowTitle('Drone Debugger')


        # Define group boxes and sub-labels
        self.groups = {

            'GPS Module': {
                'box': QGroupBox('GPS Module Status:'),
                'labels':{
                    'Satellites in View': QLabel('Satellites in View: N/A'),
                    'Latitude': QLabel('Latitude: N/A'),
                    'Longitude': QLabel('Longitude: N/A'),
                    'Time': QLabel('Time: N/A'),
                    'Date': QLabel('Date: N/A')
                }
            },

            'Compass Module': {
                'box': QGroupBox('Compass Module Status:'),
                'labels':{
                    'Heading': QLabel('Heading: N/A'),
                    'Heading Difference': QLabel('Heading Difference: N/A'),
                    'Offset X': QLabel('Offset X: N/A'),
                    'Offset Y': QLabel('Offset Y: N/A'),
                    'Offset Z': QLabel('Offset Z: N/A'),
                    'Scale X': QLabel('Scale X: N/A'),
                    'Scale Y': QLabel('Scale Y: N/A'),
                    'Scale Z': QLabel('Scale Z: N/A'),
                }
            },

            'Planner Module': {
                'box': QGroupBox('Planner Module Status:'),
                'labels':{
                    'Current Session ID': QLabel('Current Session ID: N/A'),
                    'Current Pair ID': QLabel('Current Pair ID: N/A'),
                    'Current Mission State': QLabel('Current Mission State: N/A'),
                    'Current Mission Request Type': QLabel('Current Mission Request Type: N/A'),
                    'Next Waypoint': QLabel('Next Waypoint: N/A'),
                    'Next Waypoint Latitude': QLabel('Next Waypoint Latitude: N/A'),
                    'Next Waypoint Longitude': QLabel('Next Waypoint Longitude: N/A'),
                    'Looping Waypoints': QLabel('Looping Waypoints: N/A'),
                    'Distance to Next Waypoint': QLabel('Distance to Next Waypoint: N/A'),
                    'Desired Heading': QLabel('Desired Heading: N/A'),
                }
            },

            'Sensor Module': {
                'box': QGroupBox('Sensor Module Status:'),
                'labels': {
                    'Temperature': QLabel('Temperature: N/A'),
                    'pH': QLabel('pH: N/A'),
                    'Salinity': QLabel('Salinity: N/A'),
                    'Wind Speed': QLabel('Wind Speed: N/A'),
                    'Wind Direction': QLabel('Wind Direction: N/A'),
                    'Battery Charge': QLabel('Battery Charge: N/A'),
                }
            },

            'Network Module': {
                'box': QGroupBox('Network Module Status:'),
                'labels': {
                    'Modem Activity': QLabel('Modem Activity: N/A'),
                    'Signal Quality': QLabel('Signal Quality: N/A'),
                    'Network State': QLabel('Network State: N/A'),
                    'Loops Before Transmission': QLabel('Loops Before Transmission: N/A'),
                    
                    'Send Array': QLabel('==Send Array=='),

                    'Send Time': QLabel ('Send Time: N/A'),
                    'Send Session ID': QLabel('Send Session ID: N/A'),
                    'Send Pair ID': QLabel('Send Pair ID: N/A'),
                    'Send GPS Working': QLabel('Send GPS Working: N/A'),
                    'Send Compass Working': QLabel('Send Compass Working: N/A'),
                    'Send Latitude': QLabel('Send Latitude: N/A'),
                    'Send Longitude': QLabel('Send Longitude: N/A'),
                    'Send Heading': QLabel('Send Heading: N/A'),
                    'Send Second': QLabel('Send Second: N/A'),
                    'Send Minute': QLabel('Send Minute: N/A'),
                    'Send Hour': QLabel('Send Hour: N/A'),
                    'Send Day': QLabel('Send Day: N/A'),
                    'Send Month': QLabel('Send Month: N/A'),
                    'Send Year': QLabel('Send Year: N/A'),
                    'Send State': QLabel('Send State: N/A'),
                    'Send Waypoint Index': QLabel('Send Waypoint Index: N/A'),
                    'Send Temperature': QLabel('Send Temperature: N/A'),
                    'Send pH': QLabel('Send pH: N/A'),
                    'Send Salinity': QLabel('Send Salinity: N/A'),
                    'Send Wind Speed': QLabel('Send Wind Speed: N/A'),
                    'Send Wind Direction': QLabel('Send Wind Direction: N/A'),
                    'Send Battery Charge': QLabel('Send Battery Charge: N/A'),
                    'Send IMEI': QLabel('Send IMEI: N/A'),
                    
                    'Receive Array': QLabel('==Receive Array=='),
                    'Session Results': QLabel('Session Results: N/A'),
                    'Receive Time': QLabel ('Receive Time: N/A'),
                    'Receive Session ID': QLabel('Receive Session ID: N/A'),
                    'Receive Pair ID': QLabel('Receive Pair ID: N/A'),
                    'Receive Request Type': QLabel('Receive Request Type: N/A'),
                    'Receive Loop Waypoints': QLabel('Receive Loop Waypoints: N/A'),
                    'Receive Waypoint Count': QLabel('Receive Waypoint Count: N/A'),
                    'Received Waypoints': QLabel('Received Waypoints: N/A'),
                }
            },

        }

        # Set layouts for group boxes and add labels
        for module_name, module_info in self.groups.items():
            if module_name != 'Network Module':  # All but network module
                box = module_info['box']
                group_layout = QVBoxLayout()
                box.setLayout(group_layout)
                for label in module_info['labels'].values():
                    group_layout.addWidget(label)
                self.leftLayout.addWidget(box)

        # Add network module to the right layout
        network_box = self.groups['Network Module']['box']
        network_layout = QVBoxLayout()
        network_box.setLayout(network_layout)
        for label in self.groups['Network Module']['labels'].values():
            network_layout.addWidget(label)
        self.rightLayout.addWidget(network_box)

        # Add left and right layouts to the master layout
        self.masterLayout.addLayout(self.leftLayout, 1)  # Optionally set stretch factors
        self.masterLayout.addLayout(self.rightLayout, 1)

        # Set fixed width for the left panel
        self.leftLayout.setContentsMargins(0, 0, 0, 0)
        for widget in self.leftLayout.children():
            if isinstance(widget, QGroupBox):
                widget.setFixedWidth(700)

    def updateData(self, data):
        if ':' in data:
            key, value = data.split(':', 1)
            key = key.strip()
            value = value.strip()

            if key == 'update send time':
                current_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
                self.groups['Network Module']['labels']['Send Time'].setText(f"Send Time: {current_time}")

            if key == 'update receive time':
                current_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
                self.groups['Network Module']['labels']['Receive Time'].setText(f"Receive Time: {current_time}")


            elif key == "Received Waypoints":
                layout = self.receivedWaypointsGroupBox.layout()
                # Clear previous labels
                while layout.count():
                    child = layout.takeAt(0)
                    if child.widget():
                        child.widget().deleteLater()
                # Parse and add new labels for each waypoint
                waypoints = re.findall(r'\{(\d+), (\d+)\}', value)
                for index, (lat, lon) in enumerate(waypoints, start=1):
                    layout.addWidget(QLabel(f"WP {index}: Latitude: {lat}"))
                    layout.addWidget(QLabel(f"WP {index}: Longitude: {lon}"))
            else:
                # Update other data in the respective modules
                for module, info in self.groups.items():
                    labels = info['labels']
                    if key in labels:
                        labels[key].setText(f"{key}: {value}")

if __name__ == '__main__':
    data = get_params()
    if data:
        baud_rate = data['baud_rate']
        debug_log_number = data['debug_log_number']
        serial_port = data['serial_port']
        if path_option == 1:
            path = data['path_option_1']
        elif path_option == 2:
            path = data['path_option_2']
        elif path_option == 3:
            path = data['path_option_3']
        file_name = data['file_name']
        file_extension = data['file_extension']
        full_path = path + file_name + str(debug_log_number) + file_extension
        data['debug_log_number'] += 1  # Increment log number for next use
        with open(json_file_path, 'w') as file:
            json.dump(data, file, indent=4)
        app = QApplication(sys.argv)
        ex = DroneDebuggerGUI(serial_port, baud_rate, full_path)
        ex.show()
        sys.exit(app.exec_())
    else:
        print("Failed to load configuration data.")
