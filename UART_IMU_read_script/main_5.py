from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice

import struct

start_var = 0
float_var1 = 0.0
float_var2 = 0.0
float_var3 = 0.0

end_var = 0

app = QtWidgets.QApplication([])
ui = uic.loadUi("design.ui")
ui.setWindowTitle("SerialGUI")
serial = QSerialPort()
serial.setBaudRate(115200)
portList = []
ports = QSerialPortInfo().availablePorts()
for port in ports:
    portList.append(port.portName())
ui.comList.addItems(portList)


def on_read():
    #   Ждем поступления ровно 13 байт данных:
    while serial.bytesAvailable() > 12:
        # Чтение первого байта (uint8_t)
        start_byte = serial.read(1)
        start_var = struct.unpack('B', start_byte)[0]
        # Проверка, равен ли первый байт числу 255
        if start_var == 255:
            #   Чтение следующих 12 байт
            #   (4 байта float + 4 байта float + 4 байта float)
            data_bytes = serial.read(12)
            # Распаковка данных
            float_var1, float_var2, float_var3 =\
                struct.unpack('fff', data_bytes)

            start_var = format(start_var, '6')
            float_var1 = format(float_var1, '6.2f')
            float_var2 = format(float_var2, '6.2f')
            float_var3 = format(float_var3, '6.2f')

            ui.label_6.setText(str(start_var))
            ui.label_7.setText(str(float_var1))
            ui.label_8.setText(str(float_var2))
            ui.label_9.setText(str(float_var3))
            ui.label_10.setText(str(end_var))
        else:
            serial.flush()


def on_open():
    serial.setPortName(ui.comList.currentText())
    serial.open(QIODevice.ReadOnly)


def on_close():
    serial.close()


serial.readyRead.connect(on_read)
ui.openButton.clicked.connect(on_open)
ui.closeButton.clicked.connect(on_close)
ui.show()
app.exec()


"""
            # Чтение следующих 4 байт (float)
            float_bytes1 = serial.read(4)
            float_var1 = struct.unpack('f', float_bytes1)[0]
            ui.label_2.setText(str(float_var1))
            # Чтение следующих 4 байт (float)
            float_bytes2 = serial.read(4)
            float_var2 = struct.unpack('f', float_bytes2)[0]
            ui.label_3.setText(str(float_var2))
            # Чтение следующих 2 байт (uint16_t)
            crc_bytes = serial.read(2)
            crcvar_16 = struct.unpack('H', crc_bytes)[0]
            # Чтение последнего байта (uint8_t)
            end_byte = serial.read(1)
            end_var = struct.unpack('B', end_byte)[0]
"""
