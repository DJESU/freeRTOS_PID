from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QSlider, QLCDNumber
from pyqtgraph import PlotWidget
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import numpy as np
import serial
from pyqtgraph.ptime import time
import time
import struct

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        #Load the UI Page
        uic.loadUi('ventanaqt.ui', self)

        self.stm = serial.Serial('com7', 115200)

        pen = pg.mkPen(color=(255, 0, 0))
        pen2 = pg.mkPen(color=(0, 255, 0))
        pen3 = pg.mkPen(color=(0, 0, 255))

        self.data1 = np.zeros(2000)
        self.graphWidget.setBackground('w')
        self.g_Layout = self.graphWidget

        self.p1 = self.g_Layout.addPlot()
        self.curve1 = self.p1.plot(self.data1, pen = pen)

        self.data2 = np.zeros(2000)
        self.curve2 = self.p1.plot(self.data2, pen =  pen2)
        self.data3 = np.zeros(2000)
        self.curve3 = self.p1.plot(self.data3, pen =  pen3)

        self.slider = self.findChild(QSlider,"horizontalSlider")
        self.slider2 = self.findChild(QSlider,"horizontalSlider_2")
        self.slider3 = self.findChild(QSlider,"horizontalSlider_3")
        self.slider4 = self.findChild(QSlider,"horizontalSlider_4")

        #LCD display
        self.lcd = self.findChild(QLCDNumber, "lcdNumber")
        self.lcd_2 = self.findChild(QLCDNumber, "lcdNumber_2")
        self.lcd_3 = self.findChild(QLCDNumber, "lcdNumber_3")
        self.lcd_4 = self.findChild(QLCDNumber, "lcdNumber_4")

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(5)

    # update all plots
    def update(self):
        self.update1()
        self.update2()
        self.update3()
        self.updateSlider()

    def update1(self):
        self.data1[:-1] = self.data1[1:]
        a = self.stm.readline()

        if(int(a[0]) == 82):
            a1 = int.from_bytes(a[1:3], byteorder = 'little', signed = True)
            x1 = (int(a1)*130)/100
            #print(x1)
            self.data1[-1] = x1
            self.curve1.setData(self.data1)

    def update2(self):
        self.data2[:-1] = self.data2[1:]
        a = self.stm.readline()

        if(int(a[0]) == 77):
            a1 = int.from_bytes(a[1:3], byteorder = 'little', signed = True)
            print(a1)
            x1 = (int(a1)*130)/100
            #print(x1)
            self.data2[-1] = x1
            self.curve2.setData(self.data2)

    def update3(self):
        self.data3[:-1] = self.data3[1:]
        a = self.stm.readline()
        #sprint(a)

        if(int(a[0]) == 89):
            a1 = int.from_bytes(a[1:3], byteorder = 'little', signed = True)

            x1 = (int(a1)*130)/100
            #print(x1)
            self.data3[-1] = x1
            self.curve3.setData(self.data3)

    def updateSlider(self):
        self.lcd.display(self.slider.value())
        self.lcd_2.display(self.slider2.value())
        self.lcd_3.display(self.slider3.value())
        self.lcd_4.display( (int(self.slider4.value())*130)/100 )

        result = self.slider.value()

        bytes_res = result.to_bytes(2, 'little')
        bytes_id = 11;
        bytes_id = bytes_id.to_bytes(1,'little')

        result2 = self.slider2.value()
        bytes_res2 = result2.to_bytes(2, 'little')
        bytes_id2 = 12;
        bytes_id2 = bytes_id2.to_bytes(1,'little')

        result3 = self.slider3.value()
        bytes_res3 = result3.to_bytes(2, 'little')
        bytes_id3 = 13;
        bytes_id3 = bytes_id3.to_bytes(1,'little')

        result4 = self.slider4.value()
        bytes_res4 = result4.to_bytes(2, 'little')
        bytes_id4 = 14;
        bytes_id4 = bytes_id4.to_bytes(1,'little')

        bytes_sum = bytes_id + bytes_res+ bytes_id2 + bytes_res2 + bytes_id3 + bytes_res3 + bytes_id4 + bytes_res4;
        self.stm.write(bytes_sum)


def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
