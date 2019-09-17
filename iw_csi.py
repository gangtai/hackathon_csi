from pyqtgraph.Qt import QtGui
import csi.csi as csi

class MainWindow(object):

    def __init__(self):
        self.app = QtGui.QApplication([])
        self.w = QtGui.QWidget()
        self.csi_w1 = csi.CSIWidget(title = 'CSI')

        self.layout = QtGui.QVBoxLayout()
        self.layout.addLayout(self.csi_w1.h1)
        self.w.setLayout(self.layout)
    
    def start(self):
        ## Display the widget as a new window
        self.w.show()
        self.app.exec_()
    
    def stop(self):
        self.csi_w1.stop_func()
        return
    
if __name__ == '__main__':
    mw = MainWindow()
    mw.start()
    mw.stop()




