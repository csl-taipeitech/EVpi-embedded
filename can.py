from socketcan import CanRawSocket, CanFrame

CAN_CHANNEL = 'can0'

class LEDCan:
    def __init__(self, channel) -> None:
        self.can = CanRawSocket(interface=channel)

    # Led Left
    def Left(self) -> int:
        msg = CanFrame(can_id=516, data=bytes([0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        self.can.send(msg)
        return 0

    # Led Right
    def Right(self) -> int:
        msg = CanFrame(can_id=516, data=bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        self.can.send(msg)
        return 0

    # Led Hazzard
    def Hazzard(self) -> int:
        msg = CanFrame(can_id=516, data=bytes([0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        self.can.send(msg)
        return 0

    def Non(self) -> int:
        msg = CanFrame(can_id=516, data=bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
        self.can.send(msg)
        return 0

    def TestNone(self) -> int:
        import time
        msg = CanFrame(can_id=0x201, data=bytes([1, 12, 0, 0, 0]))
        self.can.send(msg)
        # time.sleep(0.1)
        msg = CanFrame(can_id=0x200, data=bytes([0]))
        self.can.send(msg)
        return 0


    def TestLeft(self) -> int:
        import time
        msg = CanFrame(can_id=0x201, data=bytes([1, 12, 0, 0, 100]))
        self.can.send(msg)
        # time.sleep(0.1)
        msg = CanFrame(can_id=0x200, data=bytes([0]))
        self.can.send(msg)
        return 0

    def TestRight(self) -> int:
        import time
        msg = CanFrame(can_id=0x201, data=bytes([1, 12, 0, 100, 0]))
        self.can.send(msg)
        # time.sleep(0.1)
        msg = CanFrame(can_id=0x200, data=bytes([0]))
        self.can.send(msg)
        return 0

def test(led):
    # test
    import time

    print('test Left')
    led.Left()
    time.sleep(1)

    print('test Right')
    led.Right()
    time.sleep(1)

    print('test Hazzard')
    led.Hazzard()
    time.sleep(1)

if __name__ == '__main__':
    led = LEDCan(channel=CAN_CHANNEL)
    test(led)
