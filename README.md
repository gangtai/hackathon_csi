# IW Spectrum

Draw spectrum & CSI

## Getting Started
### 1. Clone iw-spectrum project
```
git clone --recurse-submodules ssh://git@gitlab.iw.moxa.com:30/Joey_Peng/iw-spectrum.git
```

### 2. Install python3 and necessary modules
```
sudo apt-get install python3 python3-pip
sudo pip3 install PyQt5 pyqtgraph paramiko scp
```

### 3. Initialize submodules
```
cd iw-spectrum
(optional)
git submodule init
git submodule update
```

### 4. Execute iw_spectrum
```
python3 iw_spectrum.py
```

![](images/spectrum_lsdk.PNG)


# IW CSI
```
python3 iw_csi.py
```
![](images/csi.PNG)