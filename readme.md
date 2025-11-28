# Project Installation Guide

Follow the steps below to install all required dependencies and run the script successfully.

---

## 📥 1. download the project

## 📦 2. (Optional but recommended) Create a virtual environment

This keeps the project’s dependencies isolated from your system:

### **macOS/Linux**

```bash
python3 -m venv venv
source venv/bin/activate
```

### **Windows**

```bash
python -m venv venv
venv\Scripts\activate
```

---

## 📚 3. Install required dependencies

The project includes a `requirements.txt` file.
Install everything with:

```bash
pip install -r requirements.txt
```

This ensures all the necessary Python packages are installed.

---

## ▶️ 4. Run the script

Use:

```bash
python bridge.py
```

---

## 🔄 Updating or reinstalling dependencies

If you reinstall the project or switch environments, simply run:

```bash
pip install -r requirements.txt
```
