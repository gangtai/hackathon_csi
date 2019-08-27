from pyqtgraph.Qt import QtGui
import spectrum.spectrum as sp

class MainWindow(object):

    def __init__(self):
        self.app = QtGui.QApplication([])
        self.w = QtGui.QWidget()
        self.sp_w1 = sp.SpectrumWidget(title = 'Spectrum (RF1)', slave = 0)
        self.sp_w2 = sp.SpectrumWidget(title = 'Spectrum (RF2)', slave = 1)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addLayout(self.sp_w1.h1)
        self.layout.addLayout(self.sp_w2.h1)      
        self.w.setLayout(self.layout)

    def start(self):
        ## Display the widget as a new window
        self.w.show()
        self.app.exec_()
    
    def stop(self):
        self.sp_w1.stop_func()
        self.sp_w2.stop_func()
    
if __name__ == '__main__':
    mw = MainWindow()
    mw.start()
    mw.stop()




