# Radar Ranging Project

This project contains two programs:

### **1. `bridge.py`**

Connects **live radar data** or **saved logs** to the **TinyML model** running on a microcontroller.

### **2. `logger.py`**

Records **raw radar frames** from the radar and saves them as `.npy` files (to be replayed later using the bridge).

typically:

1. Use **logger.py** to record radar data
2. Train their TinyML model
3. Use **bridge.py** to play data into the TinyML board (live or from logs)

---

## 📦 Installation

### 1. Install Python

You need **Python 3.10 or newer**.

### 2. Install the required packages

```sh
pip install -r requirements.txt
```

---

# ▶️ 1. Logger – Record Radar Data

`logger.py` connects to a radar over serial, receives frames, and saves them into a folder as:

```
<basename>_rx0.npy
<basename>_rx1.npy
<basename>_rx2.npy
```

(one file per receiver)

This recorded data can later be replayed with `bridge.py`.

---

## 🚀 How to Run `logger.py`

### **Command**

```sh
python logger.py
```

# ▶️ 2. Bridge – Send Data to the TinyML Model

`bridge.py` sends radar frames to the tinyML board in two possible ways:

---

## 2️⃣A. Live Radar Mode

Use this mode when the radar is connected and generating data in real time.

### **Command**

```sh
python src/bridge.py --radar <RADAR_PORT> --model <MODEL_PORT>
```

### Example

```sh
python src/bridge.py --radar COM6 --model COM9
```

---

## 2️⃣ B. Log Replay Mode

Use this mode to replay `.npy` logs recorded with `logger.py`.

### **Command**

```sh
python src/bridge.py --log_folder <FOLDER> --frequency <Hz> --model <MODEL_PORT>
```

### Example

```sh
python src/bridge.py --log_folder datasets/my_log --frequency 10 --model COM9
```

The folder must contain files like:

```
something_rx0.npy  
something_rx1.npy  
something_rx2.npy
```

# 💡 Overview of the Workflow

### **1. Record data (optional, using logger.py)**

You run the radar → logger.py saves the data → used for training.

### **2. Replay data or use live radar (using bridge.py)**

bridge.py sends either real-time frames or saved logs to the microcontroller running the TinyML model.
