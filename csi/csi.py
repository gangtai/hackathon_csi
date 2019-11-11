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

# for debugging
import ptvsd

SERVER_IP = "192.168.127.225"
CSI_PORT = 3490

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
        self.p1 = self.addPlot()
        self.curve = self.p1.plot()
        self.curve2 = self.p1.plot()
        self.curve3 = self.p1.plot()
        #self.addItem(self.axis_text)
        self.vLine = pg.InfiniteLine(angle=90, movable=False)
        self.hLine = pg.InfiniteLine(angle=0, movable=False)
        self.x_format = '{:.1f}'
        self.y_format = '{:.1f}'


class CSIWidget(QtGui.QWidget):
    draw_amp_signal = QtCore.pyqtSignal()

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

        width = 200
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
        self.tb_ap = QtGui.QLineEdit(server_ip)
        self.tb_ap.setMaximumWidth(width/2)

        self.label_sta1ip = QtGui.QLabel(self)
        self.label_sta1ip.setText("STA1 IP")
        self.tb_sta1ip = QtGui.QLineEdit("192.168.127.235")
        self.tb_sta1ip.setMaximumWidth(width/2)
        self.label_sta1len = QtGui.QLabel(self)
        self.label_sta1len.setText("STA1 Len")
        self.tb_sta1len = QtGui.QLineEdit("100")
        self.tb_sta1len.setMaximumWidth(width/4)
        self.label_sta2ip = QtGui.QLabel(self)
        self.label_sta2ip.setText("STA2 IP")
        self.tb_sta2ip = QtGui.QLineEdit("")
        self.tb_sta2ip.setMaximumWidth(width/2)
        self.label_sta2len = QtGui.QLabel("")
        self.label_sta2len.setText("STA2 Len")
        self.tb_sta2len = QtGui.QLineEdit("")
        self.tb_sta2len.setMaximumWidth(width/4)
        self.label_sta3ip = QtGui.QLabel(self)
        self.label_sta3ip.setText("STA3 IP")
        self.tb_sta3ip = QtGui.QLineEdit("")
        self.tb_sta3ip.setMaximumWidth(width/2)
        self.label_sta3len = QtGui.QLabel("")
        self.label_sta3len.setText("STA3 Len")
        self.tb_sta3len = QtGui.QLineEdit("")
        self.tb_sta3len.setMaximumWidth(width/4)

        self.training_btn = QtGui.QPushButton('Training\nStart')
        self.testing_btn = QtGui.QPushButton('Test\nStart')
        self.training_btn.setMaximumWidth(200)
        self.testing_btn.setMaximumWidth(200)
        self.testing_btn.setEnabled(False)
        self.testing_btn.setCheckable(1)
        self.testing_btn.setChecked(0)
        self.training_btn.clicked.connect(self.train_start_func)
        self.testing_btn.clicked.connect(self.test_func)

        # Layout
        self.lo_ap = QtGui.QHBoxLayout()
        self.lo_ap.addWidget(self.label_ap)
        self.lo_ap.addWidget(self.tb_ap)

        self.lo_sta1ip = QtGui.QHBoxLayout()
        self.lo_sta1ip.addWidget(self.label_sta1ip)
        self.lo_sta1ip.addWidget(self.tb_sta1ip)
        self.lo_sta1len = QtGui.QHBoxLayout()
        self.lo_sta1len.addWidget(self.label_sta1len)
        self.lo_sta1len.addWidget(self.tb_sta1len)

        self.lo_sta2ip = QtGui.QHBoxLayout()
        self.lo_sta2ip.addWidget(self.label_sta2ip)
        self.lo_sta2ip.addWidget(self.tb_sta2ip)
        self.lo_sta2len = QtGui.QHBoxLayout()
        self.lo_sta2len.addWidget(self.label_sta2len)
        self.lo_sta2len.addWidget(self.tb_sta2len)

        self.lo_sta3ip = QtGui.QHBoxLayout()
        self.lo_sta3ip.addWidget(self.label_sta3ip)
        self.lo_sta3ip.addWidget(self.tb_sta3ip)
        self.lo_sta3len = QtGui.QHBoxLayout()
        self.lo_sta3len.addWidget(self.label_sta3len)
        self.lo_sta3len.addWidget(self.tb_sta3len)

        self.lo_train_test_button = QtGui.QHBoxLayout()
        self.lo_train_test_button.addWidget(self.training_btn)
        self.lo_train_test_button.addWidget(self.testing_btn)

        self.v1 = QtGui.QVBoxLayout()
        self.v1.addLayout(self.lo_ap)
        self.v1.addLayout(self.lo_sta1ip)
        self.v1.addLayout(self.lo_sta1len)
        self.v1.addLayout(self.lo_sta2ip)
        self.v1.addLayout(self.lo_sta2len)
        self.v1.addLayout(self.lo_sta3ip)
        self.v1.addLayout(self.lo_sta3len)
        self.v1.addLayout(self.lo_train_test_button)

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
        self.h1.addLayout(self.v1)
        self.h1.addLayout(self.v2)
        
        self.pause = False
        self.pause_btn.clicked.connect(self.pause_func)
        self.start_btn.clicked.connect(self.start_func)
        self.stop_btn.clicked.connect(self.stop_func)

        self.timer = perpetualTimer(0.5,self.timer_cb)
        self.q0 = queue.Queue(maxsize = -1)
        self.q1 = queue.Queue(maxsize = -1)
        self.q2 = queue.Queue(maxsize = -1)
        self.draw_amp_signal.connect(self.draw_amp_cb)



    def table_init(self):
        table = QtGui.QTableWidget()

        font = QtGui.QFont('微軟雅黑', 13)
        font.setBold(True)  #設置字體加粗
        table.horizontalHeader().setFont(font) #設置表頭字體 
            
        table.setFrameShape(QtGui.QFrame.NoFrame)  ##設置無表格的外框
        table.horizontalHeader().setFixedHeight(80) ##設置表頭高度
        table.setFixedHeight(300) ##設置表頭高度
        #table.horizontalHeader().setStretchLastSection(True) ##設置最後一列拉伸至最大
        table.horizontalHeader().setResizeMode(0, QtGui.QHeaderView.Stretch)
        table.horizontalHeader().setResizeMode(1, QtGui.QHeaderView.Stretch)
        table.horizontalHeader().setResizeMode(2, QtGui.QHeaderView.Stretch)
        table.horizontalHeader().setResizeMode(3, QtGui.QHeaderView.Stretch)
        table.setColumnCount(3)
        table.setRowCount(4)
        
        table.setHorizontalHeaderLabels(['MAC address','Status','Correlation'])#設置表頭文字

        newItem = QtGui.QTableWidgetItem("00:90:E8:00:00:01")
        table.setItem(0, 0, newItem)
        newItem = QtGui.QTableWidgetItem("00:90:E8:00:00:02")
        table.setItem(1, 0, newItem)
        newItem = QtGui.QTableWidgetItem("00:90:E8:00:00:03")
        table.setItem(2, 0, newItem)
        newItem = QtGui.QTableWidgetItem("00:90:E8:00:00:04")
        table.setItem(3, 0, newItem)

        table.setItem(0, 2, QtGui.QTableWidgetItem("0"))
        table.setItem(1, 2, QtGui.QTableWidgetItem("0"))
        table.setItem(2, 2, QtGui.QTableWidgetItem("0"))
        table.setItem(3, 2, QtGui.QTableWidgetItem("0"))

        table.setItem(0, 1, QtGui.QTableWidgetItem())
        table.item(0, 1).setBackground(QtGui.QColor(255,0,0))
        table.setItem(1, 1, QtGui.QTableWidgetItem())
        table.item(1, 1).setBackground(QtGui.QColor(255,0,0))
        table.setItem(2, 1, QtGui.QTableWidgetItem())
        table.item(2, 1).setBackground(QtGui.QColor(255,0,0))
        table.setItem(3, 1, QtGui.QTableWidgetItem())
        table.item(3, 1).setBackground(QtGui.QColor(255,0,0))

        table.horizontalHeader().setResizeMode(0, QtGui.QHeaderView.ResizeToContents)
        table.horizontalHeader().setResizeMode(1, QtGui.QHeaderView.ResizeToContents)
        table.horizontalHeader().setResizeMode(2, QtGui.QHeaderView.ResizeToContents)
        table.horizontalHeader().setResizeMode(3, QtGui.QHeaderView.ResizeToContents)
        self.table = table


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
        self.timer.cancel()
        os.system('killall -9 fping')

    def draw_amp_cb(self):

        self.csi_amp.curve.clear()
        self.csi_amp.curve2.clear()
        self.csi_amp.curve3.clear()

        if(self.q0.qsize()>0):
            data = self.q0.get(False)
            csi = self.data_to_csi(data)
            self.draw_amp(csi,0)

        if(self.q1.qsize()>0):   
            data = self.q1.get(False)
            csi = self.data_to_csi(data)
            self.draw_amp(csi,1)

        if(self.q2.qsize()>0):
            data = self.q2.get(False)
            csi = self.data_to_csi(data)
            self.draw_amp(csi,2)

    def timer_cb(self):
        print("q0 ",self.q0.qsize())
        print("q1 ",self.q1.qsize())
        print("q2 ",self.q2.qsize())
        self.draw_amp_signal.emit()

    def draw_amp(self, csi, idx):

        x_idx, y_amp = self._csi.cal_amp(csi)
        self.csi_amp.p1.setRange(xRange=[0, max(x_idx)])

        if(idx == 0):
            self.csi_amp.curve.clear()
            self.csi_amp.curve.setData(x_idx, y_amp,pen='r')
        if(idx == 1):
            self.csi_amp.curve2.clear()
            self.csi_amp.curve2.setData(x_idx, y_amp,pen='g')
        if(idx == 2):
            self.csi_amp.curve3.clear()
            self.csi_amp.curve3.setData(x_idx, y_amp,pen='b')

    def data_to_csi(self, data):
        csi_status = data[0]
        csi_matrix = data[1]

        if (self.nr_idx < csi_status['nr'] and self.nc_idx < csi_status['nc']):
            csi = csi_matrix[self.nr_idx][self.nc_idx]
        else:
            csi = csi_matrix[0][0]

        return csi

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
            if (self.pause):
                return
            csi_status = data[0]
            csi = self.data_to_csi(data)
            if (len(csi)==114):
                label = csi_status['payload_len']-88
                queue_idx = label%3
                if(queue_idx == 0):
                    self.q0.put(data,False)
                elif(queue_idx == 1):
                    self.q1.put(data,False)
                else:
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
