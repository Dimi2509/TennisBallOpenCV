import serial


class Serial(object):
    def __init__(self, port='/dev/ttyUSB0', baud=9600, timeout=1):
        # initialize serial communication
        self.ser = serial.Serial(port, baud, timeout=timeout)
        

    def send_data(self, centerpoint):
        # format tuple to string x.y format
        centerpoint = f"{centerpoint[0]}.{centerpoint[1]}"
        # send encoded data as bits for serial
        self.ser.write(centerpoint.encode('utf-8'))
        

