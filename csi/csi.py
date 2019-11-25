import os
import socket
import struct
import math
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from scipy.stats import norm
import queue  
from threading import Timer,Thread,Event
import threading
from csi.real_simu import Classification
import scipy 
# for debugging
import ptvsd
import time
import pandas as pd 

SERVER_IP = "192.168.127.225"

#STA1_IP = "192.168.127.235"
#STA1_LEN = "100"

CSI_PORT = 3490
TRAIN_LEN = 100
TEST_LEN = 1
# typedef struct
# {
#     uint64_t tstamp;         /* h/w assigned time stamp */
    
#     uint16_t channel;        /* wireless channel (represented in Hz)*/
#     uint8_t  chanBW;         /* channel bandwidth (0->20MHz,1->40MHz)*/

#     uint8_t  rate;           /* transmission rate*/
#     uint8_t  nr;             /* number of receiving antenna*/
#     uint8_t  nc;             /* number of transmitting antenna*/
#     uint8_t  num_tones;      /* number of tones (subcarriers) */
#     int8_t  noise;          /* noise floor (to be updated)*/

#     uint8_t  phyerr;          /* phy error code (set to 0 if correct)*/

#     uint8_t    rssi;         /*  rx frame RSSI */
#     uint8_t    rssi_0;       /*  rx frame RSSI [ctl, chain 0] */
#     uint8_t    rssi_1;       /*  rx frame RSSI [ctl, chain 1] */
#     uint8_t    rssi_2;       /*  rx frame RSSI [ctl, chain 2] */

#     uint16_t   payload_len;  /*  payload length (bytes) */
#     uint16_t   csi_len;      /*  csi data length (bytes) */
#     uint16_t   buf_len;      /*  data length in buffer */
# }csi_struct;

# typedef struct
# {
#     int real;
#     int imag;
# }COMPLEX;




class perpetualTimer():

   def __init__(self,t,hFunction):
      self.t=t
      self.hFunction = hFunction
      self.thread = Timer(self.t,self.handle_function)

   def handle_function(self):
      self.hFunction()
      self.thread = Timer(self.t,self.handle_function)
      self.thread.start()

   def start(self):
      self.thread.start()

   def cancel(self):
      self.thread.cancel()


class IwCSI(QtCore.QThread):
    update = QtCore.pyqtSignal(tuple)

    def __init__(self, server_ip = SERVER_IP):
        QtCore.QThread.__init__(self)
        self.recv = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.server_ip = server_ip

    def stop(self):
        self.recv = 0
        print('wait thread join...')
        self.wait()
        self.stop_udp()
        self.sock.close()
        print('stop thread successfully')

    def recv_data(self):
        skip = 0
        self.recv = 1
        while (self.recv == 1):
            try:
                data, addr = self.sock.recvfrom(40960)
            except socket.timeout:
                # print('socket recv timeout!')
                pass
            except Exception as e:
                print(e)
            else:
                try:
                    # print('receive data, len={}'.format(len(data)))
                
                    # skip += 1
                    # if (skip % 200 != 0):
                    #     continue
                    csi_status, csi_matrix = self.process_data(data)
                    self.update.emit((csi_status, csi_matrix))
                except Exception as e:
                    print(e)
 

    def cal_amp_phase(self, csi):
        try:
            x_idx = [i for i in range(len(csi))]
            y_amp = 10*np.log10(np.absolute(csi))
            y_phase = np.angle(csi)
            return (x_idx, y_amp, y_phase)
        except Exception as e:
            print(e)

    def cal_amp(self, csi):
        try:
            x_idx = [i for i in range(len(csi))]
            y_amp = 10*np.log10(np.absolute(csi))
            return (x_idx, y_amp)
        except Exception as e:
            print(e)

    def process_data(self, data):
        try:
            pos = 0
            # csi_struct
            (tstamp, channel, chanBW, rate, nr, nc, num_tones, noise, phyerr, rssi, rssi_0, rssi_1, rssi_2, pad, payload_len, csi_len, buf_len) = \
                struct.unpack_from('>QH5Bb6B3H', data, pos)
            pos += 32

            # COMPLEX csi_matrix[nr_idx][nc_idx][k] => csi_matrix[3][3][114]
            csi_matrix = []
            for nr_idx in range(3):
                c = []
                for nc_idx in range(3):
                    raw_csi = struct.unpack_from('>{}i'.format(num_tones*2), data, pos)
                    pos += 114*2*4
                    real = np.array(raw_csi[0::2])
                    imag = np.array(raw_csi[1::2])
                    k = real + 1j*imag
                    c.append(k)
                csi_matrix.append(c)
            
            csi_status = {'tstamp':tstamp, 'channel':channel, 'chanBW':chanBW, 'rate':rate, 'nr':nr, 'nc':nc,\
                            'num_tones':num_tones, 'noise':noise, 'phyerr':phyerr,\
                            'rssi':rssi, 'rssi_0':rssi_0, 'rssi_1':rssi_1, 'rssi_2':rssi_2,\
                            'payload_len':payload_len, 'csi_len':csi_len, 'buf_len':buf_len}
            # print('csi_status:{}'.format(csi_status))
            return (csi_status, csi_matrix)

        except Exception as e:
            print(e)
    
    def trigger_udp(self):
            #XXX: just to trigger UDP CSI packets, don't use 8, 10, 16
            START_CSI = bytearray([1, 0, 1, 0])
            self.sock.sendto(START_CSI, (self.server_ip, CSI_PORT))

    def stop_udp(self):
        #XXX: just to trigger UDP CSI packets, don't use 8, 10, 16
        STOP_CSI = bytearray([0, 0, 1, 0])
        self.sock.sendto(STOP_CSI, (self.server_ip, CSI_PORT))

    def run(self):
        self.trigger_udp()
        self.recv_data()


class CSIPlot(pg.GraphicsWindow):
    def __init__(self, parent = None):
        super(CSIPlot, self).__init__(parent)
        #self.showGrid(y=1, alpha=1)
        self.p1 = self.addPlot(title="CSI amplitude")
        self.p1.setRange(xRange=[0, 114])
        self.p1.setRange(yRange=[0, 1])
        self.p1.showGrid(y=1, alpha=1)
        self.p1.addLegend()
        self.curve = self.p1.plot(name="STA 1")

        self.curve2 = self.p1.plot(name="STA 2")
        self.curve3 = self.p1.plot(name="STA 3")
        self.curve_valid = self.p1.plot(name="Valid STA")
        #self.addItem(self.axis_text)
        self.vLine = pg.InfiniteLine(angle=90, movable=False)
        self.hLine = pg.InfiniteLine(angle=0, movable=False)
        self.x_format = '{:.1f}'
        self.y_format = '{:.1f}'


class CSIWidget(QtGui.QWidget):
    draw_amp_signal = QtCore.pyqtSignal(tuple)
    draw_center_signal = QtCore.pyqtSignal()

    def __init__(self, parent = None, title = '', server_ip = SERVER_IP):
        super(CSIWidget, self).__init__(parent)
        self.started = False
        self.server_ip = server_ip
        self.tstamp_t = QtGui.QLineEdit()
        self.channel_t = QtGui.QLineEdit()
        self.bw_t = QtGui.QLineEdit()
        self.rate_t = QtGui.QLineEdit()
        self.rssi_t = QtGui.QLineEdit()
        self.rssi0_t = QtGui.QLineEdit()
        self.rssi1_t = QtGui.QLineEdit()
        self.rssi2_t = QtGui.QLineEdit()
        self.nf_t = QtGui.QLineEdit()
        self.pause_btn = QtGui.QPushButton('Pause')

        self.nr_idx = 0
        self.nc_idx = 0
        self.nrnc_box = QtGui.QHBoxLayout()
        self.nr_cb = QtGui.QComboBox()
        self.nc_cb = QtGui.QComboBox()
        self.nr_cb.addItems(['nr_index', '0', '1', '2'])
        self.nc_cb.addItems(['nc_index', '0', '1', '2'])
        self.nr_cb.currentIndexChanged.connect(self.nr_change)
        self.nc_cb.currentIndexChanged.connect(self.nc_change)
        self.nrnc_box.addWidget(self.nr_cb)
        self.nrnc_box.addWidget(self.nc_cb)

        self.ip_box = QtGui.QHBoxLayout()
        #self.server_ip_t = QtGui.QLineEdit(server_ip)
        self.start_btn = QtGui.QPushButton('Start')
        self.stop_btn = QtGui.QPushButton('Stop')
        #self.ip_box.addWidget(self.server_ip_t)
        self.ip_box.addWidget(self.start_btn)
        self.ip_box.addWidget(self.stop_btn)

        width = 250
        label_width = 80
        self.tstamp_t.setMaximumWidth(width)
        self.channel_t.setMaximumWidth(width)
        self.bw_t.setMaximumWidth(width)
        self.rate_t.setMaximumWidth(width)
        self.rssi_t.setMaximumWidth(width)
        self.rssi0_t.setMaximumWidth(width)
        self.rssi1_t.setMaximumWidth(width)
        self.rssi2_t.setMaximumWidth(width)
        self.nf_t.setMaximumWidth(width)
        self.pause_btn.setMaximumWidth(width)

        self.nr_cb.setMaximumWidth(width/2)
        self.nc_cb.setMaximumWidth(width/2)

        #self.server_ip_t.setMaximumWidth(width/2)
        self.start_btn.setMaximumWidth(width/4)
        self.stop_btn.setMaximumWidth(width/4)

        #self.v1 = QtGui.QVBoxLayout()
        #self.v1.addWidget(self.tstamp_t)
        #self.v1.addWidget(self.channel_t)
        #self.v1.addWidget(self.bw_t)
        #self.v1.addWidget(self.rate_t)
        #self.v1.addWidget(self.rssi_t)
        #self.v1.addWidget(self.rssi0_t)
        #self.v1.addWidget(self.rssi1_t)
        #self.v1.addWidget(self.rssi2_t)
        #self.v1.addWidget(self.nf_t)
        #self.v1.addWidget(self.pause_btn)
        #self.v1.addLayout(self.nrnc_box)
        #self.v1.addLayout(self.ip_box)

        #FRED: elements for demo
        self.label_ap = QtGui.QLabel(self)
        self.label_ap.setText("AP IP")
        self.label_ap.setFixedWidth(label_width)
        self.tb_ap = QtGui.QLineEdit(server_ip)
        self.tb_ap.setMaximumWidth(width/2)

        self.label_sta1ip = QtGui.QLabel(self)
        self.label_sta1ip.setText("STA1 IP")
        self.label_sta1ip.setFixedWidth(label_width)
        self.tb_sta1ip = QtGui.QLineEdit("192.168.127.235")
        self.tb_sta1ip.setMaximumWidth(width/2)

        self.label_sta1len = QtGui.QLabel(self)
        self.label_sta1len.setText("STA1 Len")
        self.label_sta1len.setFixedWidth(label_width)
        self.tb_sta1len = QtGui.QLineEdit("1")
        self.tb_sta1len.setMaximumWidth(width/6)

        self.label_sta2ip = QtGui.QLabel(self)
        self.label_sta2ip.setText("STA2 IP")
        self.label_sta2ip.setFixedWidth(label_width)
        self.tb_sta2ip = QtGui.QLineEdit("192.168.127.236")
        self.tb_sta2ip.setMaximumWidth(width/2)

        self.label_sta2len = QtGui.QLabel(self)
        self.label_sta2len.setText("STA2 Len")
        self.label_sta2len.setFixedWidth(label_width)
        self.tb_sta2len = QtGui.QLineEdit("2")
        self.tb_sta2len.setMaximumWidth(width/6)
        

        self.label_sta3ip = QtGui.QLabel(self)
        self.label_sta3ip.setText("STA3 IP")
        self.label_sta3ip.setFixedWidth(label_width)
        self.tb_sta3ip = QtGui.QLineEdit("192.168.127.237")
        self.tb_sta3ip.setMaximumWidth(width/2)

        self.label_sta3len = QtGui.QLabel(self)
        self.label_sta3len.setText("STA3 Len")
        self.label_sta3len.setFixedWidth(label_width)
        self.tb_sta3len = QtGui.QLineEdit("3")
        self.tb_sta3len.setMaximumWidth(width/6)

        self.training_btn = QtGui.QPushButton('Training\nStart')
        self.testing_btn = QtGui.QPushButton('Test\nStart')
        self.training_btn.setFixedWidth(100)
        self.testing_btn.setFixedWidth(100)
        self.testing_btn.setEnabled(False)
        self.testing_btn.setCheckable(1)
        self.testing_btn.setChecked(0)
        self.training_btn.clicked.connect(self.train_start_func)
        self.testing_btn.clicked.connect(self.test_func)

        # adjustable
        self.tb_train_time = QtGui.QLineEdit("10")
        self.tb_train_time.setMaximumWidth(width/6)
        self.tb_fping_interval = QtGui.QLineEdit("50")
        self.tb_fping_interval.setMaximumWidth(width/6)
        self.cb_tracking = QtGui.QCheckBox("Track")

        # Layout
        self.lo_ap = QtGui.QHBoxLayout()
        self.lo_ap.addWidget(self.label_ap)
        self.lo_ap.addWidget(self.tb_ap)
        self.lo_ap.addStretch(1)

        self.lo_sta1ip = QtGui.QHBoxLayout()
        self.lo_sta1ip.addWidget(self.label_sta1ip)
        self.lo_sta1ip.addWidget(self.tb_sta1ip)
        self.lo_sta1ip.addStretch(1)
        self.lo_sta1len = QtGui.QHBoxLayout()
        self.lo_sta1len.addWidget(self.label_sta1len)
        self.lo_sta1len.addWidget(self.tb_sta1len)
        self.lo_sta1len.addStretch(1)

        self.lo_sta2ip = QtGui.QHBoxLayout()
        self.lo_sta2ip.addWidget(self.label_sta2ip)
        self.lo_sta2ip.addWidget(self.tb_sta2ip)
        self.lo_sta2ip.addStretch(1)
        self.lo_sta2len = QtGui.QHBoxLayout()
        self.lo_sta2len.addWidget(self.label_sta2len)
        self.lo_sta2len.addWidget(self.tb_sta2len)
        self.lo_sta2len.addStretch(1)

        self.lo_sta3ip = QtGui.QHBoxLayout()
        self.lo_sta3ip.addWidget(self.label_sta3ip)
        self.lo_sta3ip.addWidget(self.tb_sta3ip)
        self.lo_sta3ip.addStretch(1)
        self.lo_sta3len = QtGui.QHBoxLayout()
        self.lo_sta3len.addWidget(self.label_sta3len)
        self.lo_sta3len.addWidget(self.tb_sta3len)
        self.lo_sta3len.addStretch(1)

        self.lo_train_test_button = QtGui.QHBoxLayout()
        self.lo_train_test_button.addStretch(1)
        self.lo_train_test_button.addWidget(self.training_btn)
        self.lo_train_test_button.addStretch(2)
        self.lo_train_test_button.addWidget(self.testing_btn)
        self.lo_train_test_button.addStretch(1)

        self.lo_adjustable_config = QtGui.QHBoxLayout()
        self.lo_adjustable_config.addWidget(self.tb_train_time)
        self.lo_adjustable_config.addWidget(self.tb_fping_interval)
        self.lo_adjustable_config.addWidget(self.cb_tracking)


        self.v1_widget = QtGui.QWidget()
        self.v1_widget.setFixedWidth(250)
        self.v1 = QtGui.QVBoxLayout(self.v1_widget)
        self.v1.addLayout(self.lo_ap)
        self.v1.addSpacing(10);
        self.v1.addLayout(self.lo_sta1ip)
        self.v1.addLayout(self.lo_sta1len)
        self.v1.addSpacing(10);
        self.v1.addLayout(self.lo_sta2ip)
        self.v1.addLayout(self.lo_sta2len)
        self.v1.addSpacing(10);
        self.v1.addLayout(self.lo_sta3ip)
        self.v1.addLayout(self.lo_sta3len)
        self.v1.addSpacing(10);
        self.v1.addLayout(self.lo_train_test_button)
        self.v1.addStretch(50)
        self.v1.addLayout(self.lo_adjustable_config)

        self.table_init()

        self.csi_amp = CSIPlot()

        #self.csi_amp.setWindowTitle(title + ' (Amplitude)')
        #self.csi_amp.setXRange(0, 113)
        #self.csi_amp.setYRange(0, 50)
        #self.csi_amp.setLabel('left', 'Amplitude', 'dB')
        #self.csi_amp.setLabel('bottom', 'Subcarrier Index')


        self.v2 = QtGui.QVBoxLayout()
        self.v2.addWidget(self.csi_amp)
        self.v2.addWidget(self.table)

        self.h1 = QtGui.QHBoxLayout()
        self.h1.addWidget(self.v1_widget)
        self.h1.addLayout(self.v2)
        
        self.pause = False
        self.pause_btn.clicked.connect(self.pause_func)
        self.start_btn.clicked.connect(self.start_func)
        self.stop_btn.clicked.connect(self.stop_func)

        self.q0 = queue.Queue(maxsize = -1)
        self.q1 = queue.Queue(maxsize = -1)
        self.q2 = queue.Queue(maxsize = -1)
        self.draw_amp_signal.connect(self.draw_amp_cb)
        self.draw_center_signal.connect(self.draw_center_cb)
        #self.update_table_signal.connect(self.update_table)
        self.DUT_obj_dict = {}


    def table_init(self):
        table = QtGui.QTableWidget()

        font = QtGui.QFont('�L�n����', 12)
        font.setBold(True)  #�]�m�r��[��
        table.horizontalHeader().setFont(font) #�]�m���Y�r�� 
            
        table.setFrameShape(QtGui.QFrame.NoFrame)  ##�]�m�L���檺�~��
        table.horizontalHeader().setFixedHeight(80) ##�]�m���Y����
        table.setFixedHeight(300) ##�]�m���Y����
        table.horizontalHeader().setStretchLastSection(True)
        #table.horizontalHeader().setResizeMode(0, QtGui.QHeaderView.Stretch)
        #table.horizontalHeader().setResizeMode(1, QtGui.QHeaderView.Stretch)
        #table.horizontalHeader().setResizeMode(2, QtGui.QHeaderView.Stretch)
        #table.horizontalHeader().setResizeMode(3, QtGui.QHeaderView.Stretch)
        table.horizontalHeader().setResizeMode(QtGui.QHeaderView.Stretch)
        table.verticalHeader().setDefaultSectionSize(50)
        table.setColumnCount(4)
        table.setRowCount(3)
        
        table.setHorizontalHeaderLabels(['IP address','Status','Correlation','Corr(cos)'])#�]�m���Y��r

        newItem = QtGui.QTableWidgetItem("N/A")
        table.setItem(0, 0, newItem)
        table.item(0, 0).setBackground(QtGui.QColor(255,255,0))

        newItem = QtGui.QTableWidgetItem("N/A")
        table.setItem(1, 0, newItem)
        table.item(1, 0).setBackground(QtGui.QColor(255,0,255))

        newItem = QtGui.QTableWidgetItem("N/A")
        table.setItem(2, 0, newItem)
        table.item(2, 0).setBackground(QtGui.QColor(0,255,255))
        

        table.setItem(0, 2, QtGui.QTableWidgetItem("0"))
        table.setItem(1, 2, QtGui.QTableWidgetItem("0"))
        table.setItem(2, 2, QtGui.QTableWidgetItem("0"))

        table.setItem(0, 3, QtGui.QTableWidgetItem("0"))
        table.setItem(1, 3, QtGui.QTableWidgetItem("0"))
        table.setItem(2, 3, QtGui.QTableWidgetItem("0"))

        table.setItem(0, 1, QtGui.QTableWidgetItem())
        table.item(0, 1).setBackground(QtGui.QColor(255,0,0))
        table.setItem(1, 1, QtGui.QTableWidgetItem())
        table.item(1, 1).setBackground(QtGui.QColor(255,0,0))
        table.setItem(2, 1, QtGui.QTableWidgetItem())
        table.item(2, 1).setBackground(QtGui.QColor(255,0,0))

        #table.horizontalHeader().setResizeMode(0, QtGui.QHeaderView.ResizeToContents)
        #table.horizontalHeader().setResizeMode(1, QtGui.QHeaderView.ResizeToContents)
        #table.horizontalHeader().setResizeMode(2, QtGui.QHeaderView.ResizeToContents)

        self.table = table
        self.buffer_q0 = []
        self.buffer_q1 = []
        self.buffer_q2 = []
        self.buffer_q0_corr = []
        self.buffer_q1_corr = []
        self.buffer_q2_corr = []

    def pause_func(self):
        self.pause = not self.pause

    def start_func(self):
        if (self.started == True):
            return
        #self.server_ip = self.server_ip_t.text()
        self.server_ip = self.tb_ap.text()
        self._csi = IwCSI(server_ip=self.server_ip)
        self._csi.update.connect(self.update)
        self._csi.start()
        self.started = True
        self.timer = perpetualTimer(0.5,self.timer_cb)
        self.timer.start()
        os.system('killall -9 fping')
        os.system('killall -9 fping')
        os.system('fping 192.168.127.235 -l -p 100 -b 1 > /dev/null &')
        os.system('fping 192.168.127.236 -l -p 100 -b 2 > /dev/null &')
        return

    def stop_func(self):
        if (self.started == False):
            return
        self._csi.stop()
        self.started = False
        self.test_timer.cancel()
        os.system('killall -9 fping')

    def draw_center_cb(self):
        x_idx = [i for i in range(114)]
        self.csi_amp.curve_valid.clear()
        self.csi_amp.curve_valid.setData(x_idx, self.DUT_obj_dict[0].cent_data,pen=pg.mkPen('g', width=2))
        #self.update_table()

    def draw_amp_cb(self,data):
        self.draw_amp(data[0],data[1])

    def inference(self,q):
        test_data = self.get_train_from_q(q)
        test_data_mean = np.mean(test_data, axis=0)
        test_data_mean = test_data_mean - np.min(test_data_mean)
        test_data_mean = test_data_mean / np.max(test_data_mean)

        q.queue.clear()

        dist = np.zeros(len(self.DUT_obj_dict), float)
        for i in range(len(self.DUT_obj_dict)):
            #dist[i] = scipy.spatial.distance.cosine(self.DUT_obj_dict[i].cent_data, test_data_mean)# + scipy.spatial.distance.correlation(self.DUT_obj_dict[i].cent_data, test_data_mean) + scipy.spatial.distance.euclidean(self.DUT_obj_dict[i].cent_data, test_data_mean)
            dist[i] = scipy.spatial.distance.correlation(self.DUT_obj_dict[i].cent_data, test_data_mean)

        dist0_cos = scipy.spatial.distance.cosine(self.DUT_obj_dict[0].cent_data, test_data_mean)
        min_idx = np.argmin(dist)
        #self.DUT_obj_dict[min_idx].update_cent_data(test_data_mean)

        #if (dist[min_idx]< 0.1):
        pred = self.DUT_obj_dict[min_idx].prediction()
        #else:
        #    pred = -1
        return pred, test_data_mean, dist[0], dist0_cos
        
    def test_timer_cb(self):
        print("q0 ",self.q0.qsize())
        print("q1 ",self.q1.qsize())
        print("q2 ",self.q2.qsize())
        QBUF_SIZE = 25
        CORR_BUF_SIZE = 10
        CORR_MAX = 2
        corr0 = CORR_MAX
        corr1 = CORR_MAX
        corr2 = CORR_MAX
        corr0_cos = [CORR_MAX,CORR_MAX,CORR_MAX]
        pred_avg0 = -1
        pred_avg1 = -1
        pred_avg2 = -1
        corr_avg0 = CORR_MAX
        corr_avg1 = CORR_MAX
        corr_avg2 = CORR_MAX
        test_data_mean = np.empty((3,114))

        if(self.q0.qsize() >= TEST_LEN):
            pred, test_data_mean[0,:], corr0, corr0_cos[0] = self.inference(self.q0)
            #print(len(self.buffer_q0))
            print('corr0: '+str(corr0))
            if(len(self.buffer_q0) > QBUF_SIZE):
                self.buffer_q0.pop(0)
            self.buffer_q0.append(pred)
            pred_avg0 = scipy.stats.mode(self.buffer_q0)[0][0]

            if(len(self.buffer_q0_corr) > CORR_BUF_SIZE):
                self.buffer_q0_corr.pop(0)
            self.buffer_q0_corr.append(corr0)
            corr_avg0 = np.mean(self.buffer_q0_corr)         
        
            self.draw_amp_signal.emit((test_data_mean[0,:],0))
            self.table.item(0, 3).setText(str('%.4f' % corr0_cos[0]))
            
        if(self.q1.qsize() >= TEST_LEN):
            pred, test_data_mean[1,:], corr1, corr0_cos[1] = self.inference(self.q1)
            #print(len(self.buffer_q1))
            if(len(self.buffer_q1) > QBUF_SIZE):
                self.buffer_q1.pop(0)
            self.buffer_q1.append(pred)
            pred_avg1 = scipy.stats.mode(self.buffer_q1)[0][0]

            if(len(self.buffer_q1_corr) > CORR_BUF_SIZE):
                self.buffer_q1_corr.pop(0)
            self.buffer_q1_corr.append(corr1)
            corr_avg1 = np.mean(self.buffer_q1_corr)    

            self.draw_amp_signal.emit((test_data_mean[1,:],1))
            self.table.item(1, 3).setText(str('%.4f' % corr0_cos[1]))
            
        if(self.q2.qsize() >= TEST_LEN):
            pred, test_data_mean[2,:], corr2, corr0_cos[2] = self.inference(self.q2)
            #print(len(self.buffer_q2))
            if(len(self.buffer_q2) > QBUF_SIZE):
                self.buffer_q2.pop(0)
            self.buffer_q2.append(pred)
            pred_avg2 = scipy.stats.mode(self.buffer_q2)[0][0]


            if(len(self.buffer_q2_corr) > CORR_BUF_SIZE):
                self.buffer_q2_corr.pop(0)
            self.buffer_q2_corr.append(corr2)
            corr_avg2 = np.mean(self.buffer_q2_corr)

            self.draw_amp_signal.emit((test_data_mean[2,:],2))
            self.table.item(2, 3).setText(str('%.4f' % corr0_cos[2]))
        
        corr_list = [corr_avg0,corr_avg1,corr_avg2]
        pred_list = [pred_avg0,pred_avg1,pred_avg2]
        corr_raw = [corr0,corr1,corr2]

        self.update_table_status(pred_list,corr_list,test_data_mean,corr_raw)

    def update_table_status(self,pred_list,corr_list,test_data_mean,corr_raw):
        min_idx = np.argmin(corr_list)
        for i in range(len(corr_list)):
            if(i == min_idx and pred_list[i] == 0):
                #if (corr_list[i] < 0.08):
                    #self.table.item(i, 1).setBackground(QtGui.QColor(0,255,0))
                #else:
                #    self.table.item(i, 1).setBackground(QtGui.QColor(255,0,0))
                self.table.item(i, 1).setBackground(QtGui.QColor(0,255,0))
                if (self.cb_tracking.isChecked()):
                    #print('track: update idx '+str(i)+' (corr='+str(corr_raw[i])+') to reference')
                    self.DUT_obj_dict[0].update_cent_data(test_data_mean[i,:])
                    self.draw_center_signal.emit()
            else:
                self.table.item(i, 1).setBackground(QtGui.QColor(255,0,0))

            if(corr_list[i] < 10):
                self.table.item(i, 2).setText(str('%.4f' % corr_list[i]))
            else:
                self.table.item(i, 2).setText('N/A')

    def _remove_nan(self,data):
        
        data = data.dropna(axis=0, how='any')
        
        return data

    def _remove_inf(self,data):
        
        data = data.replace([np.inf, -np.inf], np.nan)
        
        return data

    def _remove_zero(self,data):

        data = data.replace(0, np.nan)
        
        return data

    def get_train_from_q(self,q):
        len = q.qsize()
        train_data_list = []
        for i in range(len):
            data = q.get(False)
            csi = self.data_to_csi(data)
            x_idx, y_amp = self._csi.cal_amp(csi)
            train_data_list.append(y_amp)
        
        curr_data = pd.DataFrame(train_data_list)
        curr_data = self._remove_inf(curr_data)
        curr_data = self._remove_zero(curr_data)
        curr_data = self._remove_nan(curr_data)

        return curr_data.values

    def train_timer_cb(self):
        #if qsize > 100 then call train_api
        print("q0 ",self.q0.qsize())
        print("q1 ",self.q1.qsize())
        print("q2 ",self.q2.qsize())
        
        if(self.q0.qsize() > TRAIN_LEN):
            test_data_mean = self.get_train_from_q(self.q0)
            
            test_data_mean = test_data_mean - np.expand_dims(np.min(test_data_mean, axis=1), axis=-1)
            test_data_mean = test_data_mean / np.expand_dims(np.max(test_data_mean, axis=1), axis=-1)
            self.DUT_obj_dict[0] = Classification(test_data_mean, 0)
            self.draw_center_signal.emit()

        if(self.q1.qsize() > TRAIN_LEN):
            test_data_mean = self.get_train_from_q(self.q1)
            
            test_data_mean = test_data_mean - np.expand_dims(np.min(test_data_mean, axis=1), axis=-1)
            test_data_mean = test_data_mean / np.expand_dims(np.max(test_data_mean, axis=1), axis=-1)
            self.DUT_obj_dict[1] = Classification(test_data_mean, 1)
            
        if(self.q2.qsize() > TRAIN_LEN):
            test_data_mean = self.get_train_from_q(self.q2)
            
            test_data_mean = test_data_mean - np.expand_dims(np.min(test_data_mean, axis=1), axis=-1)
            test_data_mean = test_data_mean / np.expand_dims(np.max(test_data_mean, axis=1), axis=-1)
            self.DUT_obj_dict[2] = Classification(test_data_mean, 2)
        
        self.train_end()

    def draw_amp(self, amp, idx):
        x_idx = [i for i in range(114)]
        if(idx == 0):
            self.csi_amp.curve.clear()
            self.csi_amp.curve.setData(x_idx, amp,pen=QtGui.QColor(255,255,0))
        if(idx == 1):
            self.csi_amp.curve2.clear()
            self.csi_amp.curve2.setData(x_idx, amp,pen=QtGui.QColor(255,0,255))
        if(idx == 2):
            self.csi_amp.curve3.clear()
            self.csi_amp.curve3.setData(x_idx, amp,pen=QtGui.QColor(0,255,255))

    def clear_curve(self):
        self.csi_amp.curve.clear()
        self.csi_amp.curve2.clear()
        self.csi_amp.curve3.clear()

    def clear_curve_valid(self):
        self.csi_amp.curve_valid.clear()

    def clear_queue(self):
        self.q0.queue.clear()
        self.q1.queue.clear()
        self.q2.queue.clear()
    
    def clear_buffer(self):
        self.buffer_q0.clear()
        self.buffer_q1.clear()
        self.buffer_q2.clear()
        self.buffer_q0_corr.clear()
        self.buffer_q1_corr.clear()
        self.buffer_q2_corr.clear()

    def data_to_csi(self, data):
        csi_status = data[0]
        csi_matrix = data[1]

        if (self.nr_idx < csi_status['nr'] and self.nc_idx < csi_status['nc']):
            csi = csi_matrix[self.nr_idx][self.nc_idx]
        else:
            csi = csi_matrix[0][0]

        return csi

    def start_ping(self):
        self.stop_ping()
        
        if (len(self.tb_fping_interval.text())>0):
            ping_interval = self.tb_fping_interval.text()
        else:
            ping_interval = '50'

        if (len(self.tb_sta1ip.text())>0 and len(self.tb_sta1len.text())>0):
            os.system('fping '+self.tb_sta1ip.text()+' -l -p '+ping_interval+' -b '+str(self.tb_sta1len.text())+' > /dev/null &')
            print('start ping '+self.tb_sta1ip.text())

        if (len(self.tb_sta2ip.text())>0 and len(self.tb_sta2len.text())>0):
            os.system('fping '+self.tb_sta2ip.text()+' -l -p '+ping_interval+' -b '+str(self.tb_sta2len.text())+' > /dev/null &')
            print('start ping '+self.tb_sta2ip.text())

        if (len(self.tb_sta3ip.text())>0 and len(self.tb_sta3len.text())>0):
            os.system('fping '+self.tb_sta3ip.text()+' -l -p '+ping_interval+' -b '+str(self.tb_sta3len.text())+' > /dev/null &')
            print('start ping '+self.tb_sta3ip.text())

    def stop_ping(self):
        os.system('killall -9 fping')

    def update_table(self):
        if (len(self.tb_sta1ip.text())>0 and len(self.tb_sta1len.text())>0):
            self.table.item(0, 0).setText(self.tb_sta1ip.text())
        else:
            self.table.item(0, 0).setText('N/A')
        self.table.item(0, 1).setBackground(QtGui.QColor(255,0,0))
        self.table.item(0, 2).setText('')
        self.table.item(0, 3).setText('')

        if (len(self.tb_sta2ip.text())>0 and len(self.tb_sta2len.text())>0):
            self.table.item(1, 0).setText(self.tb_sta2ip.text())
        else:
            self.table.item(1, 0).setText('N/A')
        self.table.item(1, 1).setBackground(QtGui.QColor(255,0,0))
        self.table.item(1, 2).setText('')
        self.table.item(1, 3).setText('')

        if (len(self.tb_sta3ip.text())>0 and len(self.tb_sta3len.text())>0):
            self.table.item(2, 0).setText(self.tb_sta3ip.text())
        else:
            self.table.item(2, 0).setText('N/A')
        self.table.item(2, 1).setBackground(QtGui.QColor(255,0,0))
        self.table.item(2, 2).setText('')
        self.table.item(2, 3).setText('')

        #self.table.horizontalHeader().setResizeMode(0, QtGui.QHeaderView.ResizeToContents)
        #self.table.horizontalHeader().setResizeMode(1, QtGui.QHeaderView.ResizeToContents)
        #self.table.horizontalHeader().setResizeMode(2, QtGui.QHeaderView.ResizeToContents)
        
    def train_start_func(self):
        if (len(self.tb_ap.text())<=0):
            return
        if (len(self.tb_sta1ip.text())<=0 or len(self.tb_sta1len.text())<=0):
            return

        self.training_btn.setEnabled(False)
        self.training_btn.setText("Training\nProcessing")
        self.testing_btn.setEnabled(False)
        self.tb_ap.setEnabled(False)
        self.tb_sta1ip.setEnabled(False)
        self.tb_sta1len.setEnabled(False)
        self.tb_sta2ip.setEnabled(False)
        self.tb_sta2len.setEnabled(False)
        self.tb_sta3ip.setEnabled(False)
        self.tb_sta3len.setEnabled(False)
        self.clear_curve()
        self.clear_curve_valid()
        self.clear_queue()

        self._csi = IwCSI(server_ip=self.server_ip)
        self._csi.update.connect(self.update)
        self._csi.start()
        self.started = True

        self.start_ping()

        self.train_timer = Timer(int(self.tb_train_time.text()), self.train_timer_cb)
        self.train_timer.start()
        return

    def train_end(self):
        self.training_btn.setEnabled(True)
        self.training_btn.setText("Training\nComplete")
        self.testing_btn.setEnabled(True)
        self.tb_sta1ip.setEnabled(True)
        self.tb_sta1len.setEnabled(True)
        self.tb_sta2ip.setEnabled(True)
        self.tb_sta2len.setEnabled(True)
        self.tb_sta3ip.setEnabled(True)
        self.tb_sta3len.setEnabled(True)

        self.stop_ping()
        self._csi.stop()
        self.started = False

        #self.update_table()
        return

    def test_func(self):
        if (self.testing_btn.isChecked()):
            self.testing_btn.setText("Test\nStop")
            self.training_btn.setEnabled(False)
            self.tb_sta1ip.setEnabled(False)
            self.tb_sta1len.setEnabled(False)
            self.tb_sta2ip.setEnabled(False)
            self.tb_sta2len.setEnabled(False)
            self.tb_sta3ip.setEnabled(False)
            self.tb_sta3len.setEnabled(False)
            self.clear_curve()
            self.update_table()
            self.clear_queue()
            self.clear_buffer()

            self._csi = IwCSI(server_ip=self.server_ip)
            self._csi.update.connect(self.update)
            self._csi.start()
            self.started = True

            self.start_ping()

            self.test_timer = perpetualTimer(0.1,self.test_timer_cb)
            self.test_timer.start()
        else:
            self.testing_btn.setText("Test\nStart")
            self.training_btn.setEnabled(True)
            self.tb_sta1ip.setEnabled(True)
            self.tb_sta1len.setEnabled(True)
            self.tb_sta2ip.setEnabled(True)
            self.tb_sta2len.setEnabled(True)
            self.tb_sta3ip.setEnabled(True)
            self.tb_sta3len.setEnabled(True)

            self.test_timer.cancel()
            self.stop_ping()
            self._csi.stop()
            self.started = False
        return

    def draw_amp_and_display_text(self, data):

        csi_status = data[0]
        csi = self.data_to_csi(data[1])
        draw_amp(csi,0)

        chw = {0:'20', 3:'40'}
        self.tstamp_t.setText('Time Stamp = {}'.format(csi_status['tstamp']))
        self.channel_t.setText('Channel = {} MHz'.format(csi_status['channel']))
        self.bw_t.setText('Bandwidth = {} MHz'.format(chw[csi_status['chanBW']]))
        self.rate_t.setText('Rate code = 0x{:02X}'.format(csi_status['rate']))
        self.rssi_t.setText('RSSI (combiled) = {}'.format(csi_status['rssi']))
        self.rssi0_t.setText('RSSI Chain 0 = {}'.format(csi_status['rssi_0']))
        self.rssi1_t.setText('RSSI Chain 1 = {}'.format(csi_status['rssi_1']))
        self.rssi2_t.setText('RSSI Chain 2 = {}'.format(csi_status['rssi_2']))
        self.nf_t.setText('Noise = {} dBm'.format(csi_status['noise']))
 

    def update(self, data):
        try:
            csi_status = data[0]
            csi = self.data_to_csi(data)
            #print("csi_payload len:",csi_status['payload_len'])
            if (len(csi)==114):
                if(len(self.tb_sta1len.text()) > 0 and csi_status['payload_len'] == 88 + int(self.tb_sta1len.text())): # label = 1
                    self.q0.put(data,False)
                elif(len(self.tb_sta2len.text()) > 0 and csi_status['payload_len'] == 88 + int(self.tb_sta2len.text())): # label = 2
                    self.q1.put(data,False)
                elif(len(self.tb_sta3len.text()) > 0 and csi_status['payload_len'] == 88 + int(self.tb_sta3len.text())): # label = 3
                    self.q2.put(data,False)
        
        except Exception as e:
            print(e)

    def nr_change(self, i):
        nr_idx = self.nr_cb.currentText()
        if (nr_idx != 'nr_index'):
            nr_idx = int(nr_idx)
        else:
            nr_idx = 0
        self.nr_idx = nr_idx
    
    def nc_change(self, i):
        nc_idx = self.nc_cb.currentText()
        if (nc_idx != 'nr_index'):
            nc_idx = int(nc_idx)
        else:
            nc_idx = 0
        self.nc_idx = nc_idx
