# Xsens XDA Python Utilities and Integration with OpenSim's OpenSense

This repository contains scripts, virtual environment setup, and example code for working with Xsens MTw IMU devices using the Xsens Device API (XDA) in Python. It is intended for logging, parsing, and processing IMU data from Xsens' MTw. 
This repository also uses OpenSense's Python scripting to convert, calibrate and run inverse kinematics on the recorded IMU data. The resulting calibrated skeletal models and the inverse kinematic results are compatible with OpenSim's analyzing tools. 

## Folder Structure
- `mtw_receive_data.py`  
  Script to receive live data from MTw devices.
- `mtw_parse_logfile.py`  
  Script to parse MTw log files and extract data.
- `Mti_examples_for_inspiration/`  
  Original example scripts for MTi devices, made by Xsens.
- `my_python3_9_venv/`  
  Python 3.9 virtual environment with required dependencies and Xsens XDA wheel installed.
- `OpenSense/`  
  Utilities for converting and calibrating IMU data for OpenSim's OpenSense. And inverse kinematics based on IMU data script.
- `recordings/`  
  Example log files, recordings and results from Xsens Mtw devices.

## Requirements
- Linux, ubuntu 24.04 (not tested on previous versions)
- Python virtual environment with:
    - **Python 3.7–3.9** (Python 3.9 recommended)
    - Xsens MT SDK (with Python wheel for your platform)
- Conda with OpenSim package
- Xsens Mtw awinda IMUs

## IMU documentation
![1](https://github.com/user-attachments/assets/60c79a68-edf0-40b0-8eee-a99875b20d37)


## Setup Instructions

### Python virtual environment for Xsens XDA
1. **Create and activate a Python 3.9 virtual environment:**
   ```sh
   python3.9 -m venv my_python3_9_venv
   source my_python3_9_venv/bin/activate
   ```

2. **Install required Python packages:**
   ```sh
   pip install wheel
   ```

3. **Install the Xsens Device API wheel:**
   Install xsensdeviceapi wheel:
   Located in 
   Windows: <INSTALL_FOLDER>\MTSDK\Python\x64 or Win32
   Linux: <INSTALL_FOLDER>/xsens/python

    pip install xsensdeviceapi-<xda version>-cp<Python version>-none-<os type>.whl

    For example (MTSDK 2021.0.0 wheel for Python 3.9 on Linux):
    pip install /usr/local/xsens/python/xsensdeviceapi-2022.2.0-cp39-none-linux_x86_64.whl 

4. **Install system dependencies:**
   ```sh
   sudo apt-get install libpython3.9
   ```

5. **Set the library path if needed:**
   ```sh
   export LD_LIBRARY_PATH=$(python -c "import sys; print(sys.prefix)")/lib/python3.9/site-packages
   ```

### Conda environement for OpenSense library

1. **Install Conda:**
  Follow the instructions on the Anaconda website: https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html

2. **Create and activate Conda environment:**
  ```sh
  conda create -n opensim_scripting python=3.11 numpy
  conda activate opensim_scripting
  ```
3. **Libraries installation:**
  ```sh
  conda install conda-forge::simbody
  conda install conda-forge::cma
  ```

4. **OpenSim installation:**
  ```sh
  conda install -c opensim-org opensim
  ```


## Usage 
- **Alignment of IMU orientations**:
  Usually, after some time, the orientations drift. Therefore it's best to run Awinda's MtManager, add all IMUs into the interface and then do an Alignment of the Orientations. You can use the 3D view of the IMUs to check that the alignment worked. 
  The results should be better because of that. 

- **Has to be ran from root**:
  ```sh
  sudo -s
  ```

- **Activate the my_python3_9_venv**:
  ```sh
  source xda_python/my_python3_9_venv/bin/activate
  ```

- **Receiving live data:**
  ```sh
  python mtw_receive_data.py
  ```
  The script will show in the terminal how many IMUs it was ablt to connect to. When you are happy with the number of connected IMUs, you can start the recording by writting "Y" in the terminal. 
  You will also be asked to give the trial number.
  You will be able to see what sampling rate has been chosen for the trial. 
  Ctrl + C to stop the recording. The parsing is then automatically called.

- **Parsing a log file:**
  ```sh
  python mtw_parse_logfile.py recordings/MT_01200627-000.mtb
  ```

- **Post-processing environment**:
  ```sh
  conda activate opensim_scripting
  ```

- **Data conversion to OpenSim compatibility**:
  ```sh
  python OpenSense_IMUDAtaConverter.py
  ``` 
  This script will ask you to give the path to the IMU mapping xml file and the folder where the IMU data is stored. 
  It will output a number of converted motion files in .sto and will correct the timestamp with the actual computer timestamp.

- **Skeletal model calibration**:
  ```sh
  python OpenSense_CalibrateModel.py
  ``` 
  This script will ask you to give the path to the skeletal OpenSim model you want to use and the name of the model and trial number. 
  It will then output a calibrated version of that model, based on the trial data. 

- **Inverse Kinemtaics based on IMU data**:
  ```sh
  python OpenSense_OrientationTRacking.py
  ``` 
  This script will ask you to give the path to the calibrated model and the converted orientation data. It will then output a folder called IKResults with the results of the inverse kinematics it ran on the provided IMU data and the error of the computation. 

## Pipeline schematic

![2](https://github.com/user-attachments/assets/c1c270ca-25f2-4c39-81e6-c264469f5882)

## Troubleshooting

- If you get import errors for `xsensdeviceapi`, try importing the versioned module directly:
  ```python
  import xsensdeviceapi.xsensdeviceapi_py39_64 as xda
  ```
- If you get permission errors on Linux, try running your shell as root:
  ```sh
  sudo -s
  source my_python3_9_venv/bin/activate
  ```
