# NexMotion-Python_API
NexCOBOT NexMotion Python API

## Environment
* Windows 7/10

## Pre-requirement
* Install `NexMotion Studio` to get NexMotion C/C++ API and NexMotion.dll
* Create MiniBOT (or Other six-axis robot arm) Configuration from NexMotion Studio
* For operating real robot arm, do homing process in NexMotion Studio first.
* Python2.7 32bit Version (NexMotion.dll is 32bit version).

## Set up python environment using Anaconda.
Please install Anaconda3 and follow the below steps to set up python2.7 32bit environment and install necessary library.
Open `Anaconda Prompt` and type following command:
``` bash
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
