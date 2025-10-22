from pyb import UART

class Serialer:
    def __init__(self):
        self.uart = UART(3, 115200)

    def SendTrackState(self, state):
        data = f"$TRACK:{state}#"

        dataWritten = self.uart.write(data)

        if dataWritten == len(data):
            print(f"[TX] Sent: {data}")
        else:
            print(f"[TX-ERROR] Failed to send full message. Expected {len(data)}, sent {dataWritten}.")
