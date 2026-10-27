"""STS 시리즈용 최소 SDK 래퍼.

PyPI feetech-servo-sdk 1.0.0에는 sms_sts 클래스가 없어서 PacketHandler 위에 직접 만든다.
SDK 버전마다 sms_sts 유무와 함수 시그니처가 달라서, 이 프로젝트는 항상 이 래퍼만 쓴다.
"""
from scservo_sdk import PacketHandler, SCS_LOBYTE, SCS_HIBYTE, SCS_LOWORD, SCS_HIWORD, SCS_TOHOST


class sms_sts:
    def __init__(self, port):
        self.port = port
        self.ph = PacketHandler(0)   # STS 시리즈: 리틀 엔디언

    def getTxRxResult(self, res):
        return self.ph.getTxRxResult(res)

    def ping(self, scs_id):
        return self.ph.ping(self.port, scs_id)              # (model, res, err)

    def readTxRx(self, scs_id, addr, length):
        return self.ph.readTxRx(self.port, scs_id, addr, length)   # (data, res, err)

    def read1ByteTxRx(self, scs_id, addr):
        return self.ph.read1ByteTxRx(self.port, scs_id, addr)

    def read2ByteTxRx(self, scs_id, addr):
        return self.ph.read2ByteTxRx(self.port, scs_id, addr)

    def write1ByteTxRx(self, scs_id, addr, value):
        return self.ph.write1ByteTxRx(self.port, scs_id, addr, value)

    def write1ByteTxOnly(self, scs_id, addr, value):
        return self.ph.write1ByteTxOnly(self.port, scs_id, addr, value)   # 브로드캐스트용 (응답 없음)

    def ReadPosSpeed(self, scs_id):
        data, res, err = self.ph.read4ByteTxRx(self.port, scs_id, 56)   # 56: 위치, 58: 속도
        pos = SCS_TOHOST(SCS_LOWORD(data), 15)
        speed = SCS_TOHOST(SCS_HIWORD(data), 15)
        return pos, speed, res, err

    def WritePosEx(self, scs_id, position, speed, acc):
        # 41: 가속도, 42: 목표 위치, 44: 운전 시간(0), 46: 목표 속도
        if position < 0:
            position = (-position) | (1 << 15)
        data = [acc, SCS_LOBYTE(position), SCS_HIBYTE(position), 0, 0, SCS_LOBYTE(speed), SCS_HIBYTE(speed)]
        return self.ph.writeTxRx(self.port, scs_id, 41, len(data), data)
