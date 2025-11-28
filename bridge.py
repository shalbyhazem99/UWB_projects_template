import argparse
from pathlib import Path
import re
import os
import numpy as np
import serial
import time
import threading

ser_rafar = None
ser_model = None

thread_lock = threading.Lock()
thread_running = False

def send_to_model(ser_model, frame):
    if(frame.shape[0]==1536):

        ser_model.write(b"BEGIN\n")
        time.sleep(0.01)

        frame_to_send = b""
        for i in range(3):
            frame_to_send += frame[(i*512)+32:(i*512)+32+80].tobytes()
        frame_to_send += b'\n'

        print("Sending frame to model: ", frame_to_send.hex(" "))
        ser_model.write(frame_to_send)
        time.sleep(0.01)

        ser_model.write(b"END\n")
        time.sleep(0.01)
    else:
        print("Frame incorrect!!!, size: ", frame.shape[0])




def thread_log(complete_logs, log_folder, frequency, ser_model):

    delay = 1/float(frequency)

    for file_log in complete_logs:
        df_rx = []
        size = 0
        for rx_index in range(3):
            df_rx.append(np.load(os.path.join(log_folder,file_log + f"_rx{rx_index}.npy")))
            if size < len(df_rx[-1]):
                size = len(df_rx[-1])

        print(f"\nFile {file_log} started publishing!")

        for i in range(size):
            frame = b""
            ser_model.write(b"BEGIN\n")
            for rx_index in range(3):
                #frame += (b"\x00" * 2)  # header
                for c in df_rx[rx_index][i][:20]:
                    frame += complex_to_4bytes(c)
            #frame += (b"\x00" * 2)  # header
            #frame += b"\n"

            ser_model.write(frame)

            #print(frame.hex(" "))

            ser_model.write(b"END\n")

            time.sleep(delay)
        
        print(f"File {file_log} finished publishing\n")

def printDEBUG(msg):
    print("[DEBUG]: ", msg)

def radar(radar_port, model_port):

    ser_model = serial.Serial(
        port=model_port,
        baudrate=115200
    )

    ser_radar = serial.Serial(
        port=radar_port,
        baudrate=1500000
    )

    if not ser_model.is_open:
        print(f"Could not open serial port: {model_port}")
        return
    
    if not ser_radar.is_open:
        print(f"Could not open serial port: {radar_port}")
        return
    
    append_flag = False
    frame = np.empty((0), dtype = np.uint8)
    
    try:
        while True:

            if ser_radar.in_waiting:
                data = ser_radar.readline()
                ser_model.write(data)

                if not append_flag:
                    if data == b"BEGIN\n":
                        frame = np.empty((0), dtype = np.uint8)
                        append_flag=True
                else:
                    if data == b"END\n":
                        send_to_model(ser_model, frame[:-1])
                        append_flag=False
                    else:
                        frame = np.concatenate([frame, np.frombuffer(data, dtype=np.uint8)])

            if ser_model.in_waiting:
                data = ser_model.read(ser_model.in_waiting)
                ser_radar.write(data)


            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("\nManual interruption.")

    finally:
        ser_radar.close()
        ser_model.close()
        print("\nSerial connections closed.")


def complex_to_4bytes(c):
    r = np.int16(c.real)
    i = np.int16(c.imag)
    r_bytes = r.astype('>i2').tobytes()
    i_bytes = i.astype('>i2').tobytes()

    return r_bytes + i_bytes

def log(log_folder, frequency, model_port):

    ser_model = serial.Serial(
        port=model_port,
        baudrate=115200
    )

    if not ser_model.is_open:
        print(f"Could not open serial port: {model_port}")
        return

    folder = Path(log_folder)
    if not folder.exists() or not folder.is_dir():
        print(f"Unable to open folder: {folder}")
        return
    
    print("Bridge started. Press CTRL+C to exit.\n")

    npy_files = [f for f in folder.iterdir() if f.suffix == ".npy"]
    groups = {}

    for file in npy_files:
        m = re.match(r"(.*)_rx([0-2])\.npy$", file.name)
        if not m:
            continue
        base = m.group(1)
        rx_id = m.group(2)
        if base not in groups:
            groups[base] = set()
        groups[base].add(rx_id)

    complete_logs = [base for base, rxset in groups.items() if rxset == {"0", "1", "2"}]
    complete_logs.sort()

    try:
        while True:
            if ser_model.in_waiting > 0:
                msg = ser_model.readline().decode(errors="replace").strip()
                print("[MODEL]: ", msg)

                if msg == "INFO":
                    ser_model.write(b"SR250\n")
                    print("[BRIDGE]: SR250")

                elif msg == "START":
                    global thread_running

                    with thread_lock:
                        if thread_running:
                            continue
                        
                        thread_running = True

                    
                    ser_model.write(b"START\n")

                    print("Start message received!")

                    t1 = threading.Thread(target=thread_log, args=(complete_logs, log_folder, frequency, ser_model))

                    def wrapper():
                        try:
                            thread_log(complete_logs, log_folder, frequency, ser_model)
                        finally:
                            global thread_running
                            with thread_lock:
                                thread_running = False

                    t1 = threading.Thread(target=wrapper)
                    t1.start()


                else:
                    printDEBUG(msg)

            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\nManual interruption.")

    finally:
        ser_model.close()
        print("\nSerial connection closed.")
    

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="This program acts as a bridge and allows you to connect the data coming from the radar or from previously saved logs to the microcontroller on which the tinyML model runs.")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-L","--log_folder", type=str, help="Path containing the log of previously saved data.")
    group.add_argument("-R","--radar", type=str, help="Serial port to which the radar is connected.")
    parser.add_argument("-M","--model", type=str, required=True, help="Serial port to which the microcontroller on which the tinyML model is running is connected")
    parser.add_argument("-F","--frequency", type=float, help="Frequency (Hz) at which frames must be sent (required when using --log_folder).")
    args = parser.parse_args()

    if args.log_folder and args.frequency is None:
        parser.error("--frequency is required when using --log_folder")

    if args.radar:
        radar(args.radar, args.model)
    else:
        log(args.log_folder, args.frequency, args.model)