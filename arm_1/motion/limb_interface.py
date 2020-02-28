import serial


class LimbInterface:

    def __init__(self):
        pass

    def send(self, command):
        pass


class SerialLimbInterface(LimbInterface):

    def __init__(self, device_name, speed):

        self.port = serial.Serial(device_name, speed)

    def send(self, where):
        command = "{},{},{},{}".format(where['gripper'], where['elbow'], where['shoulder'], where['torso'])
        self.port.write(command.encode('ASCII'))
