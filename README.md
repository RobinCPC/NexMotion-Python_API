# NexMotion-Python_API
NexCOBOT NexMotion Python API

## Environment
* Windows 7/10

## Pre-requirement
* Install `NexMotion Studio` to get NexMotion C/C++ API and NexMotion.dll
* Create MiniBOT (or Other six-axis robot arm) Configuration from NexMotion Studio
* For operating real robot arm, do homing process in NexMotion Studio first.
* Python2.7 32bit Version (NexMotion.dll is 32bit version).

## Clone Nexmotion Python API (this repo).
In windows system, you can install `Git` to run following command in `Git Bash`.
After installing `Git`, click `Windows/start` button (on left down side of the desktop) and find `Git Bash` application.
In `Git Bash`, enter following command
``` bash
# Go to Documents folder
cd Documents
# clone NexMotion Python API
git clone https://github.com/RobinCPC/NexMotion-Python_API.git
```

## Set up python environment using virtualenv
* Install python 2.7 from python.org or use conda  
https://www.python.org/downloads/release/python-2717/

* Setup virtual environment (In MiniBOT controller)  
Open Windows CMD, then
``` batch
REM install virtualenv
pip isntall virtualenv
REM cd to Document or where you store your code
cd \Users\<your_name>\Documents\
REM create folder 
mkdir python-virtual-env
cd python-virtual-env
REM create virtual env
virtualenv minibot-env
```
* Activate environment and install necessary package for candy picking project  
Still in Windows CMD
``` batch
REM activate python virtual env
cd Documents\python-virtual-env
minibot-env\Scripts\activate
pip install -r requirements.txt
```

* Back to this repo folder and launch jupyter notebook
``` batch
cd Document\NexMotion-Python_API
jupyter notebook
```


## Or, set up python environment using Anaconda.
Please install Anaconda3 and follow the below steps to set up python2.7 32bit environment and install necessary library.
Open `Anaconda Prompt` and type following command:
``` batch
# For 64bit Windos OS, install 32bits Python
set CONDA_FORCE_32BIT=1
conda create -n py27_32 python=2.7
# To activate `py27_32`
set CONDA_FORCE_32BIT=1
activate py27_32
# Install ctypes (call NexMotion.dll in python)
conda install -c conda-forge pywin32-ctypes
# add python 32bits as a kernel for Jupyter Lab/Notebook
conda install ipykernel
python -m ipykernel install --user --name py27_32 --display-name="Python 2.7 32bits"
# For jupyter notebook demo, we can use jupyter in defualt env and use py27_32 as a kernel. 
```

note: If you need to install extra python package or run python in prompt, `set CONDA_FORCE_32BIT=1` is necessary before activate `py27_32` environment.
