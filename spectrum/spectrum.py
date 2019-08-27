#!/usr/bin/python
import socket
import struct
import math
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

# for IwSpectrumAthXk
import time
import paramiko
from scp import SCPClient
from speccy.spectrum_file import SpectrumFileReader

SERVER_IP = "192.168.127.253"
SPECTRUM_MASTER_PORT = 8001
SPECTRUM_SLAVE_PORT = 9001
NOISE_FLOOR = "0"         # 0=Use DUT's noise_floor

# typedef struct spectral_classifier_params {
#         int spectral_20_40_mode; /* Is AP in 20-40 mode? */
#         int spectral_dc_index;
#         int spectral_dc_in_mhz;
#         int upper_chan_in_mhz;
#         int lower_chan_in_mhz;
# } __ATTRIB_PACK SPECTRAL_CLASSIFIER_PARAMS;

# typedef struct spectral_samp_msg {
#         u_int32_t      signature;               /* Validates the SAMP messga */
#         u_int16_t      freq;                    /* Operating frequency in MHz */
#         u_int16_t      freq_loading;            /* How busy was the channel */
#         u_int16_t      dcs_enabled;             /* Whether DCS is enabled */
#         DCS_INT_TYPE   int_type;                /* Interference type indicated by DCS */
#         //u_int8_t       cw_interferer;           /* Indicates the presence of CW interfernce */
#         u_int8_t       macaddr[6];              /* Indicates the device interface */
#         SPECTRAL_SAMP_DATA samp_data;           /* SAMP Data */
# } __ATTRIB_PACK SPECTRAL_SAMP_MSG;

# typedef struct spectral_samp_data {
#     int16_t     spectral_data_len;                              /* indicates the bin size */
#     int16_t     spectral_rssi;                                  /* indicates RSSI */
#     int8_t      spectral_combined_rssi;                         /* indicates combined RSSI froma ll antennas */
#     int8_t      spectral_upper_rssi;                            /* indicates RSSI of upper band */
#     int8_t      spectral_lower_rssi;                            /* indicates RSSI of lower band */
#     int8_t      spectral_chain_ctl_rssi[MAX_SPECTRAL_CHAINS];   /* RSSI for control channel, for all antennas */
#     int8_t      spectral_chain_ext_rssi[MAX_SPECTRAL_CHAINS];   /* RSSI for extension channel, for all antennas */
#     u_int8_t    spectral_max_scale;                             /* indicates scale factor */
#     int16_t     spectral_bwinfo;                                /* indicates bandwidth info */
#     int32_t     spectral_tstamp;                                /* indicates timestamp */
#     int16_t     spectral_max_index;                             /* indicates the index of max magnitude */
#     int16_t     spectral_max_mag;                               /* indicates the maximum magnitude */
#     u_int8_t    spectral_max_exp;                               /* indicates the max exp */
#     int32_t     spectral_last_tstamp;                           /* indicates the last time stamp */
#     int16_t     spectral_upper_max_index;                       /* indicates the index of max mag in upper band */
#     int16_t     spectral_lower_max_index;                       /* indicates the index of max mag in lower band */
#     u_int8_t    spectral_nb_upper;                              /* Not Used */
#     u_int8_t    spectral_nb_lower;                              /* Not Used */
#     SPECTRAL_CLASSIFIER_PARAMS classifier_params;               /* indicates classifier parameters */
#     u_int16_t   bin_pwr_count;                                  /* indicates the number of FFT bins */
#     u_int8_t    bin_pwr[MAX_NUM_BINS];                          /* contains FFT magnitudes */
#     struct INTERF_SRC_RSP interf_list;                          /* list of interfernce sources */
#     int16_t noise_floor;                                        /* indicates the current noise floor */
#     u_int32_t   ch_width;                                       /* Channel width 20/40/80 MHz */
# } __ATTRIB_PACK SPECTRAL_SAMP_DATA;

class IwSpectrumLsdk(QtCore.QThread):
    update = QtCore.pyqtSignal(dict)

    def __init__(self, slave=0, server_ip = SERVER_IP):
        QtCore.QThread.__init__(self)
        self.recv = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.nf = int(NOISE_FLOOR)
        self.slave = slave
        self.server_ip = server_ip

    def stop(self):
        self.recv = 0
        print('wait thread join...')
        self.wait()
        if (self.slave == 0):
            self.stop_udp_master()
        else:
            self.stop_udp_slave()
        self.sock.close()
        print('stop thread successfully')
        

    def cal_pwr(self, bin_pwr_count, bin_pwr, lower_rssi, upper_rssi, noise_floor, ch_width, max_exp, lower_chan_in_mhz, upper_chan_in_mhz):
        # ieee 802.11 constants
        sc_wide = 0.3125  # in MHz, for 20/40MHz bandwidth
        samples = []
        if (ch_width == 0):
            freq = lower_chan_in_mhz
            # print('20MHz: center freq=%d, count=%d' % (freq, bin_pwr_count))
            sumsq_sample = 0
            for raw in bin_pwr[0:bin_pwr_count]:
                if (raw == 0):
                    sample = 1
                else:
                    sample = raw << max_exp
                sumsq_sample += sample*sample
                samples.append(sample)
            if sumsq_sample == 0:
                sumsq_sample = 1
            sumsq_sample = 10 * math.log10(sumsq_sample)

            first_sc = freq - sc_wide * (bin_pwr_count/2 + 0.5)
            pwr = {}
            for i, sample in enumerate(samples):
                subcarrier_freq = first_sc + i * sc_wide
                sigval = noise_floor + lower_rssi + 20 * math.log10(sample) - sumsq_sample
                pwr[subcarrier_freq] = sigval
                    
        elif (ch_width == 1):
            freq = (lower_chan_in_mhz + upper_chan_in_mhz)/2
            # print('40MHz: center freq=%d, count=%d' % (freq, bin_pwr_count))
            for raw in bin_pwr[0:bin_pwr_count]:
                if (raw == 0):
                    sample = 1
                else:
                    sample = raw << max_exp
                samples.append(sample)
            
            sumsq_sample_lower = 0
            for sl in samples[0:64]:
                sumsq_sample_lower += sl*sl
            # if sumsq_sample_lower == 0:
            #     sumsq_sample_lower = 1
            sumsq_sample_lower = 10 * math.log10(sumsq_sample_lower)
 
            sumsq_sample_upper = 0
            for su in samples[64:128]:
                sumsq_sample_upper += su*su
            # if sumsq_sample_upper == 0:
            #     sumsq_sample_upper = 1
            sumsq_sample_upper = 10 * math.log10(sumsq_sample_upper)

            first_sc = freq - sc_wide * (bin_pwr_count/2 + 0.5)
            pwr = {}
            for i, sample in enumerate(samples):
                subcarrier_freq = first_sc + i * sc_wide
                if i < 64:
                    sigval = noise_floor + lower_rssi + 20 * math.log10(sample) - sumsq_sample_lower
                else:
                    sigval = noise_floor + upper_rssi + 20 * math.log10(sample) - sumsq_sample_upper
                pwr[subcarrier_freq] = sigval

        else:
            print('Unknown bandwidth: %d' % ch_width)
            return

        return (freq, pwr)

    def scale_to_exp(self, scale):
        exp = 0
        while (scale):
            exp += 1
            scale = scale >> 1
        return exp

    def recv_data(self):
        skip = 0
        self.recv = 1
        while (self.recv == 1):
            try:
                data, addr = self.sock.recvfrom(2048)
                # data = self.sock.recv(2048)
            except socket.timeout:
                print('socket recv timeout!')
            else:
                pos = 0
                
                if (len(data) < 647):
                    continue
                
                skip += 1
                if (skip % 200 != 0):
                    continue

                # SPECTRAL_SAMP_MSG
                (signature, freq, freq_loading, dcs_enabled, int_type) = \
                    struct.unpack_from('>L3HL', data, pos)
                pos += 14
                macaddr = struct.unpack_from('6B', data, pos)
                pos += 6

                # SPECTRAL_SAMP_DATA
                (data_len, rssi, combined_rssi, upper_rssi, lower_rssi) = \
                    struct.unpack_from('>2h3b', data, pos)
                pos += 7
                chain_ctl_rssi = struct.unpack_from('>3B', data, pos)
                pos += 3
                chain_ext_rssi = struct.unpack_from('>3B', data, pos)
                pos += 3
                (max_scale, bwinfo, tstamp, max_index, max_mag, max_exp, last_tstamp,\
                    upper_max_index, lower_max_index, nb_upper, nb_lower) = \
                        struct.unpack_from('>Bhi2hBi2h2B', data, pos)
                pos += 22
                (spectral_20_40_mode, spectral_dc_index, spectral_dc_in_mhz, upper_chan_in_mhz, lower_chan_in_mhz) = \
                    struct.unpack_from('>5i', data, pos)
                pos += 20
                (bin_pwr_count,) = struct.unpack_from('>H', data, pos)
                pos += 2
                bin_pwr = struct.unpack_from('>512B', data, pos)
                pos += 512
                # skip interf_list
                pos += 52
                (noise_floor, ch_width) = struct.unpack_from('>hL', data, pos)

                if (self.nf == 0):  # Use DUT's noise_floor
                    nf = noise_floor
                else:
                    nf = self.nf

                # WAR: LSDK-WLAN-10.2.85 only sent max_scale, which is from max_exp
                max_exp = self.scale_to_exp(max_scale)

                (freq, pwrs) = self.cal_pwr(bin_pwr_count, bin_pwr, lower_rssi, upper_rssi, nf, ch_width, max_exp, lower_chan_in_mhz, upper_chan_in_mhz)
                data = {'freq':freq, 'pwrs':pwrs, 'ch_width':ch_width, 'noise':nf, 'tstamp':tstamp,\
                        'rssi':rssi, 'combined_rssi':combined_rssi,'lower_rssi':lower_rssi, 'upper_rssi':upper_rssi}
                self.update.emit(data)

    def trigger_udp_master(self):
        #XXX: just to trigger UDP spectral packets, don't use 8, 10, 16
        START_SCAN = bytearray([8, 0, 1, 0])
        self.sock.sendto(START_SCAN, (self.server_ip, SPECTRUM_MASTER_PORT))

    def trigger_udp_slave(self):
        #XXX: just to trigger UDP spectral packets, don't use 8, 10, 16
        START_SCAN = bytearray([8, 0, 1, 0])
        self.sock.sendto(START_SCAN, (self.server_ip, SPECTRUM_SLAVE_PORT))

    def stop_udp_master(self):
        #XXX: just to trigger UDP spectral packets, don't use 8, 10, 16
        STOP_SCAN = bytearray([10, 0, 1, 0])
        self.sock.sendto(STOP_SCAN, (self.server_ip, SPECTRUM_MASTER_PORT))

    def stop_udp_slave(self):
        #XXX: just to trigger UDP spectral packets, don't use 8, 10, 16
        STOP_SCAN = bytearray([10, 0, 1, 0])
        self.sock.sendto(STOP_SCAN, (self.server_ip, SPECTRUM_SLAVE_PORT))

    def run(self):
        if (self.slave == 0):
            self.trigger_udp_master()
        else:
            self.trigger_udp_slave()
        self.recv_data()


class IwSpectrumAthXk(QtCore.QThread):
    update = QtCore.pyqtSignal(dict)
    def __init__(self, server_ip = SERVER_IP):
        QtCore.QThread.__init__(self)
        self.recv = 0
        self.server_ip = server_ip
        self.samples_path = './spectral_samples'

    def stop(self):
        self.recv = 0
        print('wait thread join...')
        self.wait()
        print('stop thread successfully')

    def connect(self):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(self.server_ip, 22, 'root', 'moxaiw')
        self.scp = SCPClient(self.ssh.get_transport())

    def set_manual(self):
        self.ssh.exec_command('echo manual > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl')
        self.ssh.exec_command('echo 3 > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_count')

    def set_background(self):
        self.ssh.exec_command('echo background > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl')

    def trigger(self):
        self.ssh.exec_command('echo trigger > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl')

    def get_samples(self):
        self.ssh.exec_command('cat /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan0 > /tmp/spectral_samples')
        try:
            self.scp.get('/tmp/spectral_samples', self.samples_path)
            f = open(self.samples_path, 'rb')
        except:
            return None
        else:
            sample_data = f.read()
            f.close()
            return sample_data

    def update_data(self):
        samples = self.get_samples()
        for tsf, freq, noise, rssi, pwrs in SpectrumFileReader.decode(samples):
            sc_total = len(pwrs)
            ch_width = {56:0, 128:1}
            data = {'freq':freq, 'pwrs':pwrs, 'ch_width':ch_width[sc_total], 'noise':noise, 'tstamp':tsf,\
                    'rssi':rssi, 'combined_rssi':rssi,'lower_rssi':rssi, 'upper_rssi':rssi}
            self.update.emit(data)

    def run(self):
        self.connect()
        self.set_manual()
        self.recv = 1
        while (self.recv == 1):
            self.trigger()
            self.update_data()
            time.sleep(0.1)
            

class SpectrumWidget(QtGui.QWidget):
    def __init__(self, parent = None, title = '', slave=0, server_ip = SERVER_IP):
        super(SpectrumWidget, self).__init__(parent)
        self.started = False
        self.slave = slave
        self.server_ip = server_ip
        self.x_t = QtGui.QLineEdit()
        self.y_t = QtGui.QLineEdit()
        self.rssi_t = QtGui.QLineEdit()
        self.c_rssi_t = QtGui.QLineEdit()
        self.l_rssi_t = QtGui.QLineEdit()
        self.u_rssi_t = QtGui.QLineEdit()
        self.nf_t = QtGui.QLineEdit()
        self.driver_type = QtGui.QComboBox()
        self.driver_type.addItems(['LSDK', 'ATH9K/ATH10K'])
        self.pause_btn = QtGui.QPushButton('Pause')
        
        self.ip_box = QtGui.QHBoxLayout()
        self.server_ip_t = QtGui.QLineEdit(server_ip)
        self.start_btn = QtGui.QPushButton('Start')
        self.stop_btn = QtGui.QPushButton('Stop')
        self.ip_box.addWidget(self.server_ip_t)
        self.ip_box.addWidget(self.start_btn)
        self.ip_box.addWidget(self.stop_btn)

        width = 200
        self.x_t.setMaximumWidth(width)
        self.y_t.setMaximumWidth(width)
        self.rssi_t.setMaximumWidth(width)
        self.c_rssi_t.setMaximumWidth(width)
        self.l_rssi_t.setMaximumWidth(width)
        self.u_rssi_t.setMaximumWidth(width)
        self.nf_t.setMaximumWidth(width)
        self.driver_type.setMaximumWidth(width)
        self.pause_btn.setMaximumWidth(width)
        self.server_ip_t.setMaximumWidth(width/2)
        self.start_btn.setMaximumWidth(width/4)
        self.stop_btn.setMaximumWidth(width/4)

        self.v1 = QtGui.QVBoxLayout()
        self.v1.addWidget(self.x_t)
        self.v1.addWidget(self.y_t)
        self.v1.addWidget(self.rssi_t)
        self.v1.addWidget(self.c_rssi_t)
        self.v1.addWidget(self.l_rssi_t)
        self.v1.addWidget(self.u_rssi_t)
        self.v1.addWidget(self.nf_t)
        self.v1.addWidget(self.driver_type)
        self.v1.addWidget(self.pause_btn)
        self.v1.addLayout(self.ip_box)

        self.spectrum = pg.PlotWidget()
        self.spectrum.setTitle(title)
        self.spectrum.setYRange(0, -150)
        self.spectrum.showGrid(y=1, alpha=1)
        self.spectrum.setLabel('left', 'Amplitude', 'dBm')
        self.spectrum.setLabel('bottom', 'Frequency')
        self.curve = self.spectrum.plot(pen='y')
        self.vLine = pg.InfiniteLine(angle=90, movable=False)
        self.hLine = pg.InfiniteLine(angle=0, movable=False)
        self.spectrum.addItem(self.vLine, ignoreBounds=True)
        self.spectrum.addItem(self.hLine, ignoreBounds=True)
        self.vb = self.spectrum.getViewBox()
        self.proxy = pg.SignalProxy(self.spectrum.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)

        self.v2 = QtGui.QVBoxLayout()
        self.v2.addWidget(self.spectrum)

        self.h1 = QtGui.QHBoxLayout()
        self.h1.addLayout(self.v1)
        self.h1.addLayout(self.v2)
        
        self.pause = False
        self.pause_btn.clicked.connect(self.pause_func)
        self.start_btn.clicked.connect(self.start_func)
        self.stop_btn.clicked.connect(self.stop_func)
    
    def pause_func(self):
        self.pause = not self.pause

    def start_func(self):
        if (self.started == True):
            return
        self.server_ip = self.server_ip_t.text()
        rf_type = self.driver_type.currentText()
        if (rf_type == 'LSDK'):
            self.sp_rf = IwSpectrumLsdk(slave=self.slave, server_ip=self.server_ip)
        elif (rf_type == 'ATH9K/ATH10K'):
            self.sp_rf = IwSpectrumAthXk(server_ip=self.server_ip)
        
        self.sp_rf.update.connect(self.update)
        self.sp_rf.start()
        self.started = True
        return

    def stop_func(self):
        if (self.started == False):
            return
        self.sp_rf.stop()
        self.started = False
    
    def update(self, data):
        if (self.pause):
            return
        xs = np.array(list(data['pwrs'].keys()))
        ys = np.array(list(data['pwrs'].values()))
        chw = ['20', '40']
        self.spectrum.setLabel('bottom', 'Frequency = {} MHz, Bandwidth = {} MHz, Time = {}'.format(data['freq'], chw[data['ch_width']], data['tstamp']))
        self.rssi_t.setText('RSSI = {}'.format(data['rssi']))
        self.c_rssi_t.setText('Combined RSSI = {}'.format(data['combined_rssi']))
        self.l_rssi_t.setText('Lower RSSI = {}'.format(data['lower_rssi']))
        self.u_rssi_t.setText('Upper RSSI = {}'.format(data['upper_rssi']))
        self.nf_t.setText('Noise = {} dBm'.format(data['noise']))
        self.curve.setData(xs, ys)

    def mouseMoved(self, evt):
        pos = evt[0]  ## using signal proxy turns original arguments into a tuple
        if self.spectrum.sceneBoundingRect().contains(pos):
            mousePoint = self.vb.mapSceneToView(pos)
            self.x_t.setText("x = %4.1f MHz" % (mousePoint.x()))
            self.y_t.setText("y = %4.1f dBm" % (mousePoint.y()))
            self.vLine.setPos(mousePoint.x())
            self.hLine.setPos(mousePoint.y())

