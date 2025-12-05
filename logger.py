import json
import pyqtgraph as pg
import matplotlib.image as mpimg
import numpy as np
import threading
import os
import time
import serial
from scipy import signal, constants

from serial.tools import list_ports
import sounddevice as sd
from pyqtgraph.Qt.QtWidgets import (QGraphicsProxyWidget, QLineEdit, QPushButton, QLabel, 
                                    QFormLayout, QWidget, QVBoxLayout, QComboBox, QListView, 
                                    QRadioButton, QGraphicsEllipseItem, QButtonGroup, 
                                    QGraphicsRectItem, QMessageBox, QGroupBox, QHBoxLayout, QMainWindow, QCheckBox)
from pyqtgraph.Qt.QtCore import QRegExp, QSize, QThread, pyqtSignal, Qt, pyqtSlot
from pyqtgraph.Qt.QtGui import QRegExpValidator

from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from bleak import BleakScanner
from blehrm import blehrm
from collections import deque
import asyncio
import csv
import re

import src.gdx as gdx
gdx = gdx.gdx()

class SR250MateSignalProcessing(QThread):
    collection_finished = pyqtSignal(object,str)
    signalLive = pyqtSignal()
    signalRanging = pyqtSignal(int)


    def __init__(self, stop_event,fps, sr250active, sr250rangingActive):
        super().__init__()
        self.fps=fps
        self.stop_event = stop_event

        self.taps = 128
        
        self.range_bins = 120
        self.num_ant = 3
        self.bytes_per_cir = self.taps * 4 *self.num_ant

        self.frame = np.empty((0), dtype = np.uint8)
        self.append_flag = False

        self.read_ranging = sr250rangingActive

        self.datasets = "datasets"




    def run(self):
        self.start_radar() #return when total_samples_required are collected
        if not self.stop_event.is_set():
            
            t = np.linspace(0, 0.5, int(44100 * 0.5), endpoint=False)
            wave = 0.5 * np.sin(2 * np.pi * 440 * t)
            sd.play(wave, 44100)
            sd.wait()  # Wait until the sound is finished

            self.save_data()
        #self.stop_event.clear()
        print("DATA ACQUISITION FINISHED!")


    def set_parameters(self, device, samples_number, window_duration, datasets, user_id, activity, room, target_position, timestamp):
        self.samples_number = samples_number
        self.window_duration = window_duration
        self.datasets = datasets
        self.user_id = user_id
        self.activity = activity
        self.room = room
        self.target_position = target_position
        self.timestamp = timestamp
        self.total_samples_required = int(self.samples_number * (self.fps * self.window_duration) + self.fps) #add a second to have enough samples for decluttering

        self.frames  = np.zeros((self.total_samples_required, self.num_ant, self.range_bins), dtype=np.complex64)
        self.twr  = np.zeros(self.total_samples_required, dtype=np.uint16)
        self.samples_collected = 0
 
        print(f"Starting radar data acquisition for {self.user_id}...")
        print(f"Samples number: {self.samples_number}, Window duration: {self.window_duration} s")

        self.ser = serial.Serial(device, timeout=1)



    def start_radar(self):

        len_antenna = self.taps*2

        pattern = r":\s*(\d+)"

        try:
            self.ser.write(b"START")

            while not self.stop_event.is_set():

                data = self.ser.readline()

                #print(data)

                if not self.append_flag:

                    if data == b"BEGIN\n":

                        self.append_flag=True


                    elif self.read_ranging and "TWR[0].distance" in str(data):
                        try:
                            match = re.search(pattern, str(data))
                            distance_detected = np.uint16(match.group(1)) - 4630

                            self.twr[self.samples_collected] = distance_detected
                            #print(f"Distance detected: {distance_detected} cm")
                            #print(f"Sample collected: {self.samples_collected}")

                            self.signalRanging.emit(distance_detected)

                        except Exception as e:
                            print("Error parsing presence data: ", e)
                            pass
                
                else:

                    if(data == b"END\n"):
                        
                        self.frame = self.frame[:-1]

                        if(self.frame.shape[0]==self.bytes_per_cir):
                            self.frame = self.frame.view(np.int16)
                            
                            rx1 = self.frame[:len_antenna]
                            rx2 = self.frame[len_antenna:len_antenna*2]
                            rx3 = self.frame[len_antenna*2:] 
                            cir_casted_int16 = rx1[16:].reshape((len_antenna*2 - 32) // 4, 2)
                            rx1_complex = (cir_casted_int16[:, 0] + 1j * cir_casted_int16[:, 1]).astype(np.complex64)

                            cir_casted_int16 = rx2[16:].reshape((len_antenna*2 - 32) // 4, 2)
                            rx2_complex = (cir_casted_int16[:, 0] + 1j * cir_casted_int16[:, 1]).astype(np.complex64)

                            cir_casted_int16 = rx3[16:].reshape((len_antenna*2 - 32) // 4, 2)
                            rx3_complex = (cir_casted_int16[:, 0] + 1j * cir_casted_int16[:, 1]).astype(np.complex64)


                            self.frames[self.samples_collected, 0, :] = rx1_complex
                            self.frames[self.samples_collected, 1, :] = rx2_complex
                            self.frames[self.samples_collected, 2, :] = rx3_complex
                            self.signalLive.emit()
                            self.samples_collected +=1

                            if self.samples_collected == self.total_samples_required:
                                break

                        else:

                            print("Frame of shape ",self.frame.shape, "discarded")

                        self.frame = np.empty((0), dtype=np.uint8 )

                        self.append_flag = False


                        

                    else:

                        self.frame = np.concatenate([self.frame, np.frombuffer(data, dtype=np.uint8)])

            self.ser.write(b"STOP")
            #self.ser.close()
        
        except Exception as e:
            print(f"Error: {e}")

        





    def stop_acquisition(self):


        #self.save_data()

        self.stop_event.set()

        #self.ser.close()
        
        print(f"Stopping radar data acquisition for {self.user_id}...")

    

    def save_data(self):

        file_list = []

        filepath = os.path.join(self.datasets,"SR250Mate")

        if self.read_ranging:
            filepath += "_Ranging"

        if not os.path.exists(filepath):
            os.mkdir(filepath)


        filename=f"{self.user_id}_{self.activity}"

        if self.room:
            filename += f"_{self.room}"

        if self.target_position:
            filename += f"_{self.target_position}"

        filename += f"_{self.timestamp}"

        filepath = os.path.join(filepath,filename)

        print(self.frames.shape)

        if self.read_ranging:
            twr_col = self.twr.reshape(-1, 1)

        for i in range(self.num_ant):

            data=self.frames[:,i,:]
            if self.read_ranging:
                data_with_twr = np.concatenate([twr_col, data], axis=1)


            file = f"{filepath}_sr250_rx{i}.npy"

            #data = data.view(np.float32)
            np.save(file, data_with_twr if self.read_ranging else data)

            file_list.append(file)

            
        self.collection_finished.emit(file_list, "SR250Mate Ranging" if self.read_ranging else "SR250Mate")





class PolarBLESignalProcessing(QThread):
    collection_finished = pyqtSignal(object, str)
    signalLive = pyqtSignal(float, float, float)

    def __init__(self, stop_event):
        super().__init__()
        self.stop_event = stop_event
        self.address = None
        self.datasets = "datasets"
        self.acc_data = deque(maxlen=50000)
        self.recording = False
        self.timer = None
        self.blehrm_client = None
        self.connected = False

    def set_parameters(self, address, datasets):
        self.address = address
        self.datasets = datasets

    def start_recording(self, user_id, activity, room, target_position, timestamp, window_duration, samples_number):
        self.acc_data.clear()
        self.recording = True
        self.user_id=user_id
        self.activity=activity
        self.room=room
        self.target_position=target_position
        self.timestamp=timestamp
        self.window_duration=window_duration
        self.samples_number=samples_number

        total_time = window_duration * samples_number

        if self.timer:
            self.timer.stop()
            self.timer.deleteLater()
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.stop_recording)
        self.timer.start(total_time * 1000)

    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.save_data()

    def run(self):
        asyncio.run(self.collect_data())

    async def collect_data(self):
        devices = await BleakScanner.discover()
        ble_device = next((d for d in devices if d.address == self.address), None)

        if not ble_device:
            print(f"Polar {self.address} not found.")
            return

        self.blehrm_client = blehrm.create_client(ble_device)
        await self.blehrm_client.connect()
        self.connected = True
        print("Connected to Polar BLE")

        def update_acc(sample):
            ts = time.time()

            
            timestamp_polar, x, y, z = sample
            self.signalLive.emit(float(x), float(y), float(z))

            if self.recording:
                self.acc_data.append((ts, timestamp_polar, x, y, z))           

        
        await self.blehrm_client.start_acc_stream(update_acc)
        print("ACC started")

        try:
            while not self.stop_event.is_set():
                await asyncio.sleep(0.1)
        finally:
            print("Stopping Polar ACC stream...")
            try:
                await self.blehrm_client.stop_acc_stream()
                await self.blehrm_client.disconnect()
                print("Polar BLE disconnected")
            except Exception as e:
                print(f"Error during disconnect: {e}")
            self.connected = False
            self.blehrm_client = None

    def save_data(self):
        filepath = os.path.join(self.datasets,"PolarBLE")

        os.makedirs(filepath, exist_ok=True)

        filename=f"{self.user_id}_{self.activity}"

        if self.room:
            filename += f"_{self.room}"

        if self.target_position:
            filename += f"_{self.target_position}"

        filename += f"_{self.timestamp}"

        file = f"{filename}_acc.csv"

        fullpath = os.path.join(filepath, file)
        with open(fullpath, "w") as f:
            f.write("timestamp,timestamp_polar,acc_x,acc_y,acc_z\n")
            for t, tp, x, y, z in self.acc_data:
                f.write(f"{t},{tp},{x},{y},{z}\n")
        print(f"Saved Polar BLE data to {fullpath}")
        self.collection_finished.emit([fullpath], "PolarBLE")


class BreathingProcessing(QThread):
    collection_finished = pyqtSignal(object,str)
    signalLive = pyqtSignal(float, float)

    def __init__(self, stop_event):
        super().__init__()
        self.stop_event = stop_event
        self.period_in_ms = 100
        self.collection_complete=False
        self.sensors_data = [[], [], []]
        self.number_of_readings = 0
        self.datasets = "datasets"



    def run(self):
        self.collect_data() #return when total_samples_required are collected
        if not self.stop_event.is_set():
            self.save_data()
        #self.stop_event.clear()
        print("DATA ACQUISITION FINISHED!")


    def set_parameters(self, samples_number, window_duration, datasets, user_id, activity, room, target_position, timestamp):
        self.samples_number = samples_number
        self.window_duration = window_duration
        self.datasets = datasets
        self.user_id = user_id
        self.activity = activity
        self.room = room
        self.target_position = target_position
        self.timestamp = timestamp

        self.number_of_readings = int((self.samples_number * self.window_duration * 1000) / self.period_in_ms)
 
        print(f"Starting radar data acquisition for {self.user_id}...")
        print(f"Samples number: {self.samples_number}, Window duration: {self.window_duration} s")



    def collect_data(self):
        
        self.sensors_data = [[], [], []]

        self.collection_complete = False

        gdx.start(self.period_in_ms) 

        while not self.collection_complete:
            try:
                time = 0.0
                print ('Collecting Data...')

                for i in range(0,self.number_of_readings):

                    # This is where we are reading the list of measurements from the sensors.
                    measurements=gdx.read() 
                    if measurements == None: 
                        break 

                    value = measurements[0]
                    rate = measurements[1]

                    #print(str(round(time,2))+ '   '+ str(round(value,2))+ '   '+ str(round(rate,2)))

                    self.signalLive.emit(float(value), float(rate))

                    self.sensors_data[0].append(time)
                    self.sensors_data[1].append(value)
                    self.sensors_data[2].append(rate)

                    time = time+(self.period_in_ms / 1000)

                # The data collection loop is finished
                self.collection_complete = True
                gdx.stop()

            except Exception as e:
                print(f"Error: {e}")
                self.collection_complete = True
    
    def save_data(self):
        temp_path = os.path.join(self.datasets,"Breathing")

        os.makedirs(temp_path, exist_ok=True)

        filename=f"{self.user_id}_{self.activity}"

        if self.room:
            filename += f"_{self.room}"

        if self.target_position:
            filename += f"_{self.target_position}"

        filename += f"_{self.timestamp}"

        file = f"{filename}_brathing.csv"

        filepath = os.path.join(temp_path,file)

        with open(filepath, mode='w', newline='', encoding='utf-8') as file_csv:
            writer = csv.writer(file_csv)
            writer.writerow (['time','force', 'rate'])
            for i in range(len(self.sensors_data[0])):
                writer.writerow([round(self.sensors_data[0][i],2),self.sensors_data[1][i],self.sensors_data[2][i]])

        print(f"Saved Breathing BLE data to {filepath}")
        self.collection_finished.emit([filepath], "Breathing")


        
class FormLayout(QWidget):
    def __init__(self, parent_logger=None):
        super().__init__()
        self.logger = parent_logger

        layout = QFormLayout()

        self.user_textbox = QLineEdit(placeholderText = "UserID")

        layout.addRow(QLabel("UserID: "), self.user_textbox)

        self.room_textbox = QLineEdit(placeholderText = "Stanza")

        self.room_textbox.textChanged.connect(self.add_special_activities)

        layout.addRow(QLabel("Stanza: "), self.room_textbox)

        self.activity_combobox = QComboBox(placeholderText = "Attività")

        listView = QListView(self.activity_combobox)

        with open("src/logger_conf.json", 'r') as file:
            conf = json.load(file)

        for activity in conf["activities"]:

            self.activity_combobox.addItem(activity)

        self.activity_combobox.setView(listView)
        layout.addRow(QLabel("Attività: "), self.activity_combobox)

        

        self.special_combobox = QComboBox(placeholderText = "Casi Speciali")
        special_listView = QListView(self.special_combobox)

        self.special_combobox.setView(special_listView)
        self.special_combobox.setVisible(False)
        self.special_label = QLabel("Casi Speciali: ")
        self.special_label.setVisible(False)
        layout.addRow(self.special_label, self.special_combobox)

        self.activity_combobox.currentTextChanged.connect(self.toggle_special_combobox)

        regex = QRegExp("[1-9]\\d*")

        self.window_size_textbox = QLineEdit(placeholderText="Dimensione finestra (s)")
        self.window_size_textbox.setText("15")
        self.window_size_textbox.setValidator(QRegExpValidator(regex))

        layout.addRow(QLabel("Dimensione finestra (s): "), self.window_size_textbox)

        self.samples_number_textbox = QLineEdit(placeholderText = "Numero finestre")
        self.samples_number_textbox.setText("1")
        self.samples_number_textbox.setValidator(QRegExpValidator(regex))

        layout.addRow(QLabel("Numero finestre: "), self.samples_number_textbox)

        self.setLayout(layout)

        self.timerComboBox = QComboBox(placeholderText = "Timer")
        timerListView = QListView(self.timerComboBox)
        self.timerComboBox.setView(timerListView)        
        self.timerComboBox.addItems([f"{i}" for i in range(11)])
        self.timerComboBox.setCurrentIndex(0)
        self.timerLabel = QLabel("Timer: ")
        layout.addRow(self.timerLabel, self.timerComboBox) 

        checkbox_layout = QHBoxLayout()
        #checkbox_layout.setAlignment(Qt.AlignRight)
        self.sr250active = QCheckBox("SR250(Mate)")
        self.sr250active.setLayoutDirection(Qt.RightToLeft)
        self.sr250active.setStyleSheet("QCheckBox { color: crimson; }")
        self.sr250active.toggled.connect(self.init_serial_sr250) 
        self.sr250rangingActive = QCheckBox("SR250(Mate) + Ranging")
        self.sr250rangingActive.setLayoutDirection(Qt.RightToLeft)
        self.sr250rangingActive.setStyleSheet("QCheckBox { color: crimson; }") 
        self.sr250rangingActive.toggled.connect(self.init_serial_sr250)

        
        checkbox_layout.addWidget(self.sr250active)
        checkbox_layout.addStretch()
        checkbox_layout.addWidget(self.sr250rangingActive)
        checkbox_layout.addStretch()

        #checkbox_layout.setAlignment(Qt.AlignCenter)

        layout.addRow(QLabel(""), checkbox_layout)

        checkbox_gt_layout = QHBoxLayout()
        self.cardioActive = QCheckBox("Acceleration (Polar H10)")
        self.cardioActive.setLayoutDirection(Qt.RightToLeft)
        self.cardioActive.setStyleSheet("QCheckBox { color: crimson; }")
        self.cardioActive.toggled.connect(self.init_ble_cardio)
        self.breathingActive = QCheckBox("Breathing (Go Direct)")
        self.breathingActive.setLayoutDirection(Qt.RightToLeft)
        self.breathingActive.setStyleSheet("QCheckBox { color: crimson; }")
        self.breathingActive.toggled.connect(self.init_ble_breathing)

        
        checkbox_gt_layout.addWidget(self.cardioActive)
        checkbox_gt_layout.addStretch()
        checkbox_gt_layout.addWidget(self.breathingActive)
        checkbox_gt_layout.addStretch()

        layout.addRow(QLabel("Ground truth:"), checkbox_gt_layout)


    def add_special_activities(self, text):
        
        if text == "Soggiorno":

            self.special_combobox.clear()
            self.special_combobox.addItem("Disteso sul divano")

        elif text == "Camera":

            self.special_combobox.clear()
            self.special_combobox.addItem("Disteso sul letto")
        
        elif text == "Bagno":

            self.special_combobox.clear()
            self.special_combobox.addItem("Disteso in vasca")

        else:
            self.special_combobox.clear()



    def toggle_special_combobox(self, text):

        if text == "Casi Speciali":
            self.special_combobox.setVisible(True)
            self.special_label.setVisible(True)
            if self.room_textbox.text() == "":
                
                QMessageBox.warning(self, "Error", "Inserisci la stanza per visualizzare i casi speciali")
            
        else:
            self.special_combobox.setVisible(False)
            self.special_label.setVisible(False)

    
    def get_serial_ports(self, info):
        esp_port = None
        ports = list_ports.comports()
        for port in ports: 
            try: 
                ser = serial.Serial(port.device, timeout = 1) 
                ser.write(b'INFO\r\n') 
                response = ser.read(100).decode('utf-8', errors='ignore') 
                print(f"Response: {str(response)}")
                if info in response: 
                    esp_port = port.device 
                    ser.close() 
                    print(f"Found SR250 device on port {esp_port}") 
                    break 
                
            except Exception as e: 
                print(f"Error checking port {port.device}: {e}") 
                continue
        return esp_port
    

    def init_serial_sr250(self):

        if self.sr250active.isChecked() or self.sr250rangingActive.isChecked():
            esp_port = None 

            if self.sr250active.isChecked():
                esp_port = self.get_serial_ports("SR250")

            elif self.sr250rangingActive.isChecked():
                esp_port = self.get_serial_ports("Ranging")
            

            if esp_port is None:
                print("ESP device not found. Exiting.")
                if self.sr250active.isChecked():
                    self.sr250active.setChecked(False)
                elif self.sr250rangingActive.isChecked():
                    self.sr250rangingActive.setChecked(False)
            else:
                if self.sr250active.isChecked():
                    self.sr250active.setStyleSheet("QCheckBox { color: green; }") 
                elif self.sr250rangingActive.isChecked():
                    self.sr250rangingActive.setStyleSheet("QCheckBox { color: green; }") 

            self.sr250Port = esp_port

        else:
            if self.sr250active.isChecked():
                self.sr250active.setStyleSheet("QCheckBox { color: crimson; }")
            elif self.sr250rangingActive.isChecked():
                self.sr250rangingActive.setStyleSheet("QCheckBox { color: crimson; }")
            self.sr250Port = None


    def init_ble_cardio(self):
        if self.cardioActive.isChecked():
            print("Scanning for Polar BLE devices...")
            devices = asyncio.run(BleakScanner.discover(timeout=5.0))

            polar_device = next((d for d in devices if d.name and "Polar" in d.name), None)
            if polar_device:
                print(f"Found {polar_device.name} ({polar_device.address})")
                self.cardio_ble_address = polar_device.address
                self.cardioActive.setStyleSheet("QCheckBox { color: green; }")

                self.logger.start_polar_preview(self.cardio_ble_address)
            else:
                print("No Polar device found.")
                self.cardio_ble_address = None
                self.cardioActive.setChecked(False)
                self.cardioActive.setStyleSheet("QCheckBox { color: crimson; }")
        else:
            print("Polar BLE disabled.")
            self.cardio_ble_address = None
            self.cardioActive.setStyleSheet("QCheckBox { color: crimson; }")
            self.logger.stop_polar_preview()


    def init_ble_breathing(self):
        if self.breathingActive.isChecked():
            print("Scanning for Go Direct BLE devices...")

            if gdx.open(connection=self.logger.breathing_connection, device_to_open=self.logger.breathing_address):

                gdx.select_sensors([1, 2])
                info = gdx.device_info()
                print("Go Direct percentage battery: " + str(info[2]) + "%")
                
                self.breathingActive.setStyleSheet("QCheckBox { color: green; }")

            else:
                print("No Go Direct device found.")
                self.breathingActive.setChecked(False)
                self.breathingActive.setStyleSheet("QCheckBox { color: crimson; }")
                gdx.close()
        else:
            gdx.close()
            print("Go Direct BLE disabled.")
            self.breathingActive.setStyleSheet("QCheckBox { color: crimson; }")


class Logger(pg.GraphicsView):
    def __init__(self):
        super(Logger, self).__init__()

        self.visualizationMode = "Heatmap"

        #self.grid_items = []  # To track polar grid elements

        

        self.init_logger()
        l = pg.GraphicsLayout()
        self.setCentralItem(l)
        self.setBackground('w')
        self.setWindowTitle("TRUESENSE - UWB Dataset Collector")
        self.showMaximized()

        logo = mpimg.imread('./src/assets/logo.png')
        logo = logo.transpose((1, 0, 2))
        logo = np.flip(logo, 1)

        logoitem = pg.ImageItem(logo)
        logoitem.setImage(logo)

        logobox = l.addViewBox(border='w', colspan=2)
        logobox.setAspectLocked()
        logobox.addItem(logoitem)
        logobox.setFixedHeight(80)

        l.nextRow()

        title = l.addLabel("UWB Dataset Collector",  border='w', size='24pt', bold=True, color='black', anchor=(0.5,0), colspan=2)

        l.nextRow()

        self.form = FormLayout(parent_logger=self)

        container = QWidget()
        v_layout = QVBoxLayout(container)
        v_layout.addWidget(self.form)

        container.setLayout(v_layout)

        container.setObjectName("formContainer")

        container.setStyleSheet("QWidget#formContainer {background-color: white; padding: 10px;}")

        container_proxy = QGraphicsProxyWidget()
        container_proxy.setWidget(container)

        l.addItem(container_proxy, colspan=1)

        self.plot_item = pg.PlotItem()

        self.plot_item.setAspectLocked(True)
        self.plot_item.getViewBox().setRange(xRange=(-15, 500), yRange=(-40, 500), padding=0)
        self.plot_item.getViewBox().setMouseEnabled(x=False, y=False)
        #self.plot_item.setMinimumSize(400,400)
        #self.plot_item.setFixedWidth(700)
        l.layout.setColumnStretchFactor(0, 1)
        self.create_polar_grid()
        self.create_radio_buttons()
        self.plot_item.hide()

        l.addItem(self.plot_item, colspan=1, rowspan=2)


        
        
        self.button = QPushButton("Change Visualization")
        self.button.setFixedWidth(300)

        self.change_button_proxy = self.make_centered_proxy(self.button)
        
        l.addItem(self.change_button_proxy, colspan=1, row=3, col=0)
        self.button.clicked.connect(self.change_visualization)  # Connect button to function")

        self.img = []
        self.plt = []

        self.range_bins = 120
        self.alpha = 0.9
        self.normalization = (1 + self.alpha) / 2
        self.decBase = np.empty((3, self.range_bins), dtype=np.complex64)
        imgdata = np.zeros((self.range_bins, int(20 * 15)), dtype=np.float32)

        # SR250
        self.img.append(pg.ImageItem(border="w"))
        self.img[0].setImage(imgdata)
        self.img[0].setColorMap("viridis")
        self.plt.append(l.addPlot(anchor=(1, 0), colspan=1, col=1, rowspan=1, row=2))
        self.plt[0].addItem(self.img[0])
        self.plt[0].setXRange(0, self.range_bins)
        self.plt[0].setTitle("SR250", size="30pt", bold=True, color="black")
        self.plt[0].getViewBox().autoRange()

        #ACC
        self.acc_x_curve_data = []
        self.acc_y_curve_data = []
        self.acc_z_curve_data = []
        self.acc_plot = l.addPlot(anchor=(1, 0), colspan=1, col=1, rowspan=1, row=2)
        self.acc_plot.setTitle(f"ACC", size="30pt", bold=True, color="black")
        self.acc_plot.showGrid(x=True, y=True)
        self.acc_plot.setLabel('left', 'Acceleration', units='mg')
        self.acc_plot.setLabel('bottom', 'Time (s)')
        self.acc_plot.setXRange(-15, 0)
        self.acc_plot.addLegend()
        self.acc_curve_x = self.acc_plot.plot(pen=pg.mkPen('r', width=1), name='ACC X')
        self.acc_curve_y = self.acc_plot.plot(pen=pg.mkPen('g', width=1), name='ACC Y')
        self.acc_curve_z = self.acc_plot.plot(pen=pg.mkPen('b', width=1), name='ACC Z')
        self.acc_plot.hide()

        l.nextRow()

        # INFINEON
        self.img.append(pg.ImageItem(border="w"))
        self.img[1].setImage(imgdata)
        self.img[1].setColorMap("viridis")
        self.plt.append(l.addPlot(anchor=(1, 0), colspan=1, col=1, rowspan=1, row=3))
        self.plt[1].addItem(self.img[1])
        self.plt[1].setXRange(0, self.range_bins)
        self.plt[1].setTitle("Infineon", size="30pt", bold=True, color="black")
        self.plt[1].getViewBox().autoRange()
        self.plt[1].hide()

        #BREATHING
        self.breathing_curve_data = []
        self.breathing_rate_data = []
        self.breathing_plot = l.addPlot(anchor=(1, 0), colspan=1, col=1, rowspan=1, row=3)
        self.breathing_plot.setTitle("Breathing (- bpm)", size="30pt", bold=True, color="black")
        self.breathing_plot.showGrid(x=True, y=True)
        self.breathing_plot.setLabel('left', 'Forca (N)')
        self.breathing_plot.setLabel('bottom', 'Time (s)')
        self.breathing_plot.setXRange(-15, 0)
        self.breathing_curve = self.breathing_plot.plot(pen=pg.mkPen('b', width=2))
        #self.breathing_plot.enableAutoRange(axis='y', enable=True)
        self.breathing_plot.hide()

        l.nextRow()

        self.start_btn_proxy = QGraphicsProxyWidget()
        self.start_button = QPushButton("Start Collection")
        self.start_button.clicked.connect(self.start_collection)
        self.start_btn_proxy.setWidget(self.start_button)

        l.addItem(self.start_btn_proxy, colspan=1)

        self.stop_btn_proxy = QGraphicsProxyWidget()
        self.stop_button = QPushButton("Stop Collection")
        self.stop_button.clicked.connect(self.stop_collection)
        self.stop_btn_proxy.setWidget(self.stop_button)

        l.addItem(self.stop_btn_proxy, colspan=1)


        self.show()
        
    def make_centered_proxy(self,widget):
        container = QWidget()
        container.setObjectName("buttonContainer")
        container.setStyleSheet("QWidget#buttonContainer {background-color: white; padding: 10px;}")
        layout = QVBoxLayout(container)
        layout.setAlignment(Qt.AlignHCenter)
        layout.addWidget(widget)
        container.setLayout(layout)

        proxy = QGraphicsProxyWidget()
        proxy.setWidget(container)
        return proxy

        


    def init_logger(self):

        self.stop_event = threading.Event()

        """while True:

            self.device = self.init_serial_sr250()

            if(self.device):
                break

            time.sleep(10)"""

        with open('src/logger_conf.json', 'r') as f:
            self.config = json.load(f)

            self.fps = float(self.config["fps"])
            self.breathing_address = self.config["breathing_address"]
            self.breathing_connection = self.config["breathing_connection"]
            self.datasets_path = self.config["datasets_path"]

            if not os.path.exists(self.datasets_path):
                os.mkdir(self.datasets_path)
      



    def create_polar_grid(self):
        """ Draws the radar plot grid """

        self.distance = [150, 250, 350, 450] 
        self.angles = [0, 22.5, 45, 67.5, 90]  

        self.range_ticks=[[(self.distance[i],f"{str(self.distance[i]/100)}m") for i in range(len(self.distance))]]
        
        self.plot_item.getAxis('bottom').setTicks(self.range_ticks)
        self.plot_item.getAxis('left').setTicks(self.range_ticks)
        self.plot_item.getAxis('bottom').setTextPen('b')
        self.plot_item.getAxis('left').setTextPen('b')
        
        for r in self.distance:
            # Draw circles for each radial distance
            circle = QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
            circle.setPen(pg.mkPen('k', width=1))
            circle.setStartAngle(0)
            circle.setSpanAngle(-90*16)
            self.plot_item.addItem(circle)
            #self.grid_items.append(circle)
        
        # Draw angular lines
        for angle in self.angles:
            angle_rad = np.radians(angle)
            line = pg.PlotDataItem([0, np.cos(angle_rad)*max(self.distance)],
                                   [0, np.sin(angle_rad)*max(self.distance)],
                                   pen=pg.mkPen('k', width=1))
            targetItem2 = pg.TargetItem(
                pos=(np.cos(angle_rad)*max(self.distance), np.sin(angle_rad)*max(self.distance)),
                size=1,
                symbol="x",
                pen=pg.mkPen(None),
                label=f"{angle}°",
                labelOpts={
                    "offset": pg.QtCore.QPoint(15, 15),
                    "color": "#558B2F"
                }
            )
            self.plot_item.addItem(line)
            #self.grid_items.append(line)
            self.plot_item.addItem(targetItem2)
            #self.grid_items.append(targetItem2)

            rect = QGraphicsRectItem(pg.QtCore.QRectF(-20, -30, 40, 60))  # (x, y, width, height)
            rect.setPen(pg.QtGui.QPen(pg.mkColor('r'), 2))
            rect.setTransformOriginPoint(rect.rect().center())
            rect.setRotation(45)
            

            label = pg.QtWidgets.QGraphicsTextItem("RADAR")
            label.setPos(-10, 25)
            label.setRotation(45)  
            label.setTransform(pg.QtGui.QTransform().fromScale(1,-1))

        # Set the text color of the label
            label.setDefaultTextColor(pg.QtGui.QColor('red'))  # Set text color to blue

            font = pg.QtGui.QFont("Arial", 12, pg.QtGui.QFont.Bold)
            label.setFont(font)
            self.plot_item.addItem(label)
            #self.grid_items.append(label)


            self.plot_item.addItem(rect)
            #self.grid_items.append(rect)

    def change_visualization(self):
        
        if self.visualizationMode == "Heatmap": 
            self.plot_item.show()
            self.plt[0].hide()
            self.plt[1].hide()
            self.acc_plot.hide()
            self.breathing_plot.hide()
            self.visualizationMode="Grid"

        elif self.visualizationMode == "Grid":
            self.plot_item.hide()
            self.plt[0].hide()
            self.plt[1].hide()
            self.acc_plot.show()
            self.breathing_plot.show()
            self.visualizationMode = "BioSignals"
        else:
            self.plot_item.hide()
            self.plt[0].show()
            self.plt[1].hide()
            self.acc_plot.hide()
            self.breathing_plot.hide()
            self.visualizationMode="Heatmap"

    def show_polar_grid(self):
        self.plot_item.show()
        self.visualizationMode="Grid"


    def create_radio_buttons(self):
        """ Creates radio buttons and places them at appropriate grid points """
        self.positions = [
            (150, 0), (150, 22.5), (150, 45), (150, 67.5),(150, 90),  # Row 1
            (250, 0), (250, 22.5), (250, 45), (250, 67.5), (250, 90), # Row 2
            (350, 0), (350, 22.5), (350, 45), (350, 67.5), (350, 90), # Row 3
            (450, 0), (450, 22.5), (450, 45), (450, 67.5), (450, 90),   # Row 4
        ]
        self.button_group = QButtonGroup(self)

        self.last_checked = None


        # Create radio buttons for each position
        for idx, (r, angle_deg) in enumerate(self.positions, start=1):
            button = QRadioButton(str(idx))
            #button.setFixedHeight(1)
            button.setAutoExclusive(True)  #only 1 button to be checked at once
            
            self.button_group.addButton(button)
            
            # Calculate polar to Cartesian conversion for positioning the buttons
            angle_rad = np.radians(angle_deg)
            x = r * np.cos(angle_rad)-5
            y = r * np.sin(angle_rad)+12            
            
            # Map data coordinates to screen (scene) coordinates
            button_proxy = pg.QtWidgets.QGraphicsProxyWidget()
            button_proxy.setWidget(button)

            button_proxy.setTransformOriginPoint(button_proxy.rect().center())

            button_proxy.setTransform(pg.QtGui.QTransform().fromScale(1,-1))


            button_proxy.setPos(x,y)
        
            self.plot_item.addItem(button_proxy)
            #self.grid_items.append(button_proxy)
        self.button_group.buttonClicked.connect(self.toggle_radio)

    def toggle_radio(self,radio):
        
        if self.last_checked == radio:
            # Same button clicked again — toggle it off
            self.button_group.setExclusive(False)
            radio.setChecked(False)
            self.button_group.setExclusive(True)
            self.last_checked = None
        else:
            # New selection
            self.last_checked = radio
        
    def start_polar_preview(self, address):
        print("Starting Polar BLE preview thread")
        self.stop_event_ble = threading.Event()
        self.polar_ble = PolarBLESignalProcessing(stop_event=self.stop_event_ble)
        self.polar_ble.signalLive.connect(self.show_polar_acc)
        self.polar_ble.set_parameters(address=address, datasets=self.datasets_path)
        self.polar_ble.collection_finished.connect(self.save_message)
        self.polar_ble.start()

    def stop_polar_preview(self):
        print("Stopping Polar preview")
        if hasattr(self, "polar_ble"):
            self.stop_event_ble.set()

            self.polar_ble.wait()
            print("Polar BLE thread stopped")

        
    
    def start_collection(self):

        print("Start Collection")
        self.stop_event.clear()

        if (self.form.sr250active.isChecked() or self.form.sr250rangingActive.isChecked()):
            
            self.username = self.form.user_textbox.text()

            if(self.form.activity_combobox.currentText()== ""):
                QMessageBox.warning(self, "Error", "Seleziona l'attività")
                return
            else:

                self.activity = self.form.activity_combobox.currentText()

                if(self.activity == "Casi Speciali"):
                    if(self.form.special_combobox.currentText()==""):
                        QMessageBox.warning(self, "Error", "Seleziona il caso speciale!")
                        return
                    

                    else:

                        self.special_case = self.form.special_combobox.currentText()

                        self.activity += "_" + self.special_case

            self.room = self.form.room_textbox.text()

            self.samples_number = int(self.form.samples_number_textbox.text())

            self.window_duration = int(self.form.window_size_textbox.text())

            timestamp = time.strftime("%Y%m%d-%H%M%S")

            if self.visualizationMode == "Grid":
                if(self.button_group.checkedButton() is None):
                    QMessageBox.warning(self, "Error", "Seleziona la posizione del target")
                    return
                else:
                    self.selected_pos = str(self.positions[int(self.button_group.checkedButton().text())-1]).replace(" ","")
            else:
                if(self.button_group.checkedButton() is None):
                    self.selected_pos= None
                else:
                    self.selected_pos = str(self.positions[int(self.button_group.checkedButton().text())-1]).replace(" ","")

            if self.username == '':
                self.username = "GenericUser"

            time.sleep(int(self.form.timerComboBox.currentText()))
            t = np.linspace(0, 0.5, int(44100 * 0.5), endpoint=False)
            wave = 0.5 * np.sin(2 * np.pi * 440 * t)

            # Play the generated sound
            sd.play(wave, 44100)
            sd.wait()  # Wait until the sound is finished

            self.firstDec = [True, True, True]

            if self.form.sr250active.isChecked() or self.form.sr250rangingActive.isChecked():
                self.plt[0].setTitle("SR250", size="30pt", bold=True, color="black")
                self.sr250_radar = SR250MateSignalProcessing(stop_event=self.stop_event, fps = self.fps, sr250active = self.form.sr250active.isChecked(), sr250rangingActive = self.form.sr250rangingActive.isChecked())
                self.sr250_radar.collection_finished.connect(self.save_message)
                self.sr250_radar.signalLive.connect(self.show_250_hmap)
                if self.form.sr250rangingActive.isChecked():
                    self.sr250_radar.signalRanging.connect(self.show_distance_sr250)
                self.sr250_radar.set_parameters(self.form.sr250Port, self.samples_number, self.window_duration, self.datasets_path, self.username, self.activity, self.room, self.selected_pos, timestamp)
                self.dec_frames_sr250 = np.zeros((self.sr250_radar.total_samples_required,  self.sr250_radar.range_bins), dtype=np.complex64)
                self.sr250_samples_collected = 0

            if self.form.cardioActive.isChecked():
                self.polar_ble.start_recording(user_id=self.username, activity=self.activity, room=self.room, target_position=self.selected_pos, timestamp=timestamp, window_duration=self.window_duration, samples_number=self.samples_number)
            if self.form.breathingActive.isChecked():
                self.reset_breathing_plot()
                self.breathing_band = BreathingProcessing(stop_event=self.stop_event)
                self.breathing_band.collection_finished.connect(self.save_message)
                self.breathing_band.signalLive.connect(self.show_breathing_signal)
                self.breathing_band.set_parameters(self.samples_number, self.window_duration, self.datasets_path, self.username, self.activity, self.room, self.selected_pos, timestamp)    
            

            if self.form.sr250active.isChecked() or self.form.sr250rangingActive.isChecked():
                self.sr250_radar.start()
            if self.form.breathingActive.isChecked():
                self.breathing_band.start()


        else:

            QMessageBox.warning(self, "Error", "Connetti almeno un device!")
            return

    
    def save_message(self, file_list, device_name):

        msg_box =  QMessageBox()

        msg_box.setWindowTitle("Conferma salvataggio")
        msg_box.setText(f"Raccolta dati per {device_name} completata! Vuoi salvarla?")
        msg_box.setIcon(QMessageBox.Question)

        yes_btn = msg_box.addButton(QMessageBox.Save)
        no_btn = msg_box.addButton(QMessageBox.Cancel)

        msg_box.exec_()

        if(msg_box.clickedButton() == yes_btn):

            print("SAVED")

        elif(msg_box.clickedButton() == no_btn):

            for f in file_list:
                os.remove(f)
            print("DISCARDED")



    def stop_collection(self):
        print("Stop Collection")

        self.stop_event.set()

        if hasattr(self, "polar_ble"):
            self.polar_ble.stop_recording()


    def decluttering(self, cir, rx):

        cir_abs = cir

        if self.firstDec[rx]:

            self.decBase[rx] = cir_abs

            self.firstDec[rx] = False
            return cir_abs*0
        
        else:

            res = ((self.alpha * self.decBase[rx]) + (1-self.alpha)*cir_abs)
            self.decBase[rx] = res

            res = (cir_abs - res)

            return res
        
    def decluttering_alt (self, cir, rx):
        cir_abs = cir

        if self.firstDec[rx]:

            self.decBase[rx] = cir_abs

            self.firstDec[rx] = False
            return cir_abs*0
        
        else:

            res = (cir - self.decBase[rx]) * self.normalization

            self.decBase[rx] = self.decBase[rx] * self.alpha + cir * (1-self.alpha)

            return res

        
    def fft_spectrum(self, mat, range_window):
        # Calculate fft spectrum
        # mat:          chirp data
        # range_window: window applied on input data before fft

        # received data 'mat' is in matrix form for a single receive antenna
        # each row contains 'num_samples' for a single chirp
        # total number of rows = 'num_chirps'

        # -------------------------------------------------
        # Step 1 - remove DC bias from samples
        # -------------------------------------------------
        [num_chirps, num_samples] = np.shape(mat)

        # helpful in zero padding for high resolution FFT.
        # compute row (chirp) averages
        avgs = np.average(mat, 1).reshape(num_chirps, 1)

        # de-bias values
        mat = mat - avgs
        # -------------------------------------------------
        # Step 2 - Windowing the Data
        # -------------------------------------------------
        mat = np.multiply(mat, range_window)

        # -------------------------------------------------
        # Step 3 - add zero padding here
        # -------------------------------------------------
        zp1 = np.pad(mat, ((0, 0), (0, num_samples)), 'constant')

        # -------------------------------------------------
        # Step 4 - Compute FFT for distance information
        # -------------------------------------------------
        range_fft = np.fft.fft(zp1) / num_samples

        # ignore the redundant info in negative spectrum
        # compensate energy by doubling magnitude
        range_fft = 2 * range_fft[:, range(int(num_samples))]

        return range_fft
    
    @pyqtSlot()
    def show_250_hmap(self):
        self.dec_frames_sr250[self.sr250_samples_collected,:] = self.decluttering_alt(self.sr250_radar.frames[self.sr250_samples_collected,0,:], 0)
        self.sr250_samples_collected += 1
        self.img[0].setImage(np.abs(self.dec_frames_sr250).T, autolevels = True)
        self.plt[0].getViewBox().autoRange()

    @pyqtSlot(int)
    def show_distance_sr250(self, distance):
        self.plt[0].setTitle(f"SR250 ({distance} cm)", size="30pt", bold=True, color="black")

    @pyqtSlot()
    def show_250_dev_hmap(self):

        self.dec_frames_sr250dev[self.sr250dev_samples_collected,:] = self.decluttering_alt(self.sr250dev_radar.frames[self.sr250dev_samples_collected,0,:], 1)
        self.sr250dev_samples_collected += 1
        self.img[0].setImage(np.abs(self.dec_frames_sr250dev).T, autolevels = True)
        self.plt[0].getViewBox().autoRange()



    @pyqtSlot()
    def show_infineon_hmap(self):

        self.range_window = signal.windows.blackmanharris(128).reshape(1, 128)
        
        data = 2 * self.infineon_radar.frames[self.infineon_samples_collected,0,:] / 4095 -1.0 
        data = self.fft_spectrum(data, self.range_window)
        data = np.divide(data.sum(axis=0), 4)
        self.dec_frames_infineon[self.infineon_samples_collected,:] = self.decluttering_alt(data[:120],2)
        
        self.infineon_samples_collected += 1

        self.img[1].setImage(np.abs(self.dec_frames_infineon).T, autolevels = True)
        self.plt[1].getViewBox().autoRange()

    @pyqtSlot(float, float, float)
    def show_polar_acc(self, acc_x, acc_y, acc_z):
        self.acc_x_curve_data.append(acc_x)
        self.acc_y_curve_data.append(acc_y)
        self.acc_z_curve_data.append(acc_z)

        # 15 s * 200 Hz = 3000
        if len(self.acc_x_curve_data) > 3000:
            self.acc_x_curve_data = self.acc_x_curve_data[-3000:]
            self.acc_y_curve_data = self.acc_y_curve_data[-3000:]
            self.acc_z_curve_data = self.acc_z_curve_data[-3000:]
        n = len(self.acc_x_curve_data)
        t = np.linspace(-n / 200, 0, n, endpoint=False)
        self.acc_curve_x.setData(t, self.acc_x_curve_data)
        self.acc_curve_y.setData(t, self.acc_y_curve_data)
        self.acc_curve_z.setData(t, self.acc_z_curve_data)

    @pyqtSlot(float, float)
    def show_breathing_signal(self, breathing_value, rate_value):
        self.breathing_curve_data.append(breathing_value)

        if len(self.breathing_curve_data) > 75:
            self.breathing_curve_data = self.breathing_curve_data[-75:]

        if not np.isnan(rate_value):
            self.breathing_rate_data.append(rate_value)
            self.breathing_plot.setTitle(f"Breathing ({np.mean(self.breathing_rate_data):.2f} bpm)", size="30pt", bold=True, color="black")

        n = len(self.breathing_curve_data)
        t = np.linspace(-n / 5, 0, n, endpoint=False)
        self.breathing_curve.setData(t, self.breathing_curve_data)
    
    def reset_breathing_plot(self):
        self.breathing_curve_data.clear()
        self.breathing_rate_data.clear()


if __name__ == '__main__':

    app = pg.mkQApp("TRUESENSE - UWB Dataset Collector")

    with open('src/UbuntuStyle.css', 'r') as f:
        app.setStyleSheet(f.read())

    view = Logger()

        # Force fullscreen workaround
    main_window = QMainWindow()
    main_window.setCentralWidget(view)
    main_window.setWindowTitle(view.windowTitle())

    screen_geometry = app.primaryScreen().availableGeometry()
    main_window.setGeometry(screen_geometry)        # Force resize to screen size
    main_window.move(screen_geometry.topLeft())     # Move to top-left corner
    main_window.show()                              # Important: show before setting state
    main_window.setWindowState(Qt.WindowMaximized)  # Explicitly set window state

    pg.exec()
