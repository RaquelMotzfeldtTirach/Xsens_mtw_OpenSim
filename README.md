Before running the MT SDK Python example, the XDA (xsensdeviceapi)
Python interface needs to be installed using the included XDA wheel file.
Make sure to have the correct version of python and pip are installed on your machine.

Supported Python versions: 3.7.x up to 3.9.x

1. Make sure to have "wheel" installed on your machine:

pip install wheel

2. Install xsensdeviceapi wheel:

Located in 
Windows: <INSTALL_FOLDER>\MTSDK\Python\x64 or Win32
Linux: <INSTALL_FOLDER>/xsens/python

pip install xsensdeviceapi-<xda version>-cp<Python version>-none-<os type>.whl

For example (MTSDK 2021.0.0 wheel for Python 3.9 on Linux):
pip install xsensdeviceapi-2021.0.0-cp39-none-linux_x86_64.whl or

For example (MTSDK 2021.0.0 wheel for Python 3.9 on Windows):
pip install xsensdeviceapi-2021.0.0-cp39-none-win_amd64.whl

3. Now you are ready to run the MT SDK in Python

------------------------------------------------------------------------------------

Important:
On Windows, run examples from command prompt as Administrator to prevent permission issues (some examples use file operations).

------------------------------------------------------------------------------------

In case your Python IDE is unable to find the xsensdeviceapi module or if the
auto-completion does not work, try using:

import xsensdeviceapi.xsensdeviceapi_py<Python version>_64 as xda
instead of:
import xsensdeviceapi as xda

For example for Python 3.9:
import xsensdeviceapi.xsensdeviceapi_py39_64 as xda




# What did we do? 

we made a virtual environment with python 3.9 
we installed the wheel package thing
 pip install /usr/local/xsens/python/xsensdeviceapi-2025.0.0-cp39-none-linux_x86_64.whl 
 sudo apt-get install libpython3.9
export LD_LIBRARY_PATH=/home/raquel/Documents/Xsens/xda_python/my_python3_9_venv/lib/python3.9/site-packages/libpython3.9.so.1.0

but I think the 2025 version was too new for Mtw hardware! We were looking at MTi examples ...

so we did 
python -m pip uninstall /usr/local/xsens/python/xsensdeviceapi-2025.0.0-cp39-none-linux_x86_64.whl 

and then 
pip install /usr/local/xsens/python/xsensdeviceapi-2022.2.0-cp39-none-linux_x86_64.whl 
I couldn't find a python example for Mtw

but on github: https://github.com/jiminghe/Xsens_MTw_XDA_Receive/blob/master/xdamtwreceive.py
installed keyboard
Changed the import to fit the linux import stuff 

Got a root error, does not solve it with sudo before python
but this solves it 
sudo -s

then re-activate the my_python3_9_venv (source xda_python/my_python3_9_venv/bin/activate)
and then just run the code with python !! 
