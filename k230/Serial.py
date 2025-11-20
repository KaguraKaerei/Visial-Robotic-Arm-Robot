from machine import UART, FPIOA

# ========================= k230串口配置 ========================= #

fpioa = FPIOA()

# 主控板通信串口
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
fpioa.set_function(50, FPIOA.UART3_TXD)
fpioa.set_function(51, FPIOA.UART3_RXD)

uartMain = UART(
    UART.UART2, 
    baudrate=115200, 
    bits=UART.EIGHTBITS, 
    parity=UART.PARITY_NONE, 
    stop=UART.STOPBITS_ONE, 
)

# OpenMV通信串口
uartOpenMV = UART(
    UART.UART3, 
    baudrate=115200, 
    bits=UART.EIGHTBITS, 
    parity=UART.PARITY_NONE, 
    stop=UART.STOPBITS_ONE,
)

# 串口初始化函数
def Serial_Init():
    uartMain.init(
        baudrate=115200, 
        bits=UART.EIGHTBITS, 
        parity=UART.PARITY_NONE, 
        stop=UART.STOPBITS_ONE, 
    )
    
    uartOpenMV.init(
        baudrate=115200, 
        bits=UART.EIGHTBITS, 
        parity=UART.PARITY_NONE, 
        stop=UART.STOPBITS_ONE,
    )

# ========================= k230与主控板通信 ========================= #

# 发送寻迹状态到主控板
def SendTrackState(state):
    data = f"$TRACK:{state}#"
    dataWritten = uartMain.write(data)

    if dataWritten == len(data):
        print(f"[TX] Sent: {data}")
    else:
        print(f"[TX-ERROR] Failed to send full message. Expected {len(data)}, sent {dataWritten}.")

def SendCmd(cmd: str):
    dataWritten = uartMain.write(cmd)
    
    if dataWritten == len(cmd):
        print(f"[TX] Sent: {cmd}")

    else:
        print(f"[TX-ERROR] Failed to send full message. Expected {len(cmd)}, sent {dataWritten}.")

# ========================= k230与OpenMV通信 ========================= #

# 接收OpenMV发送的寻迹状态
def ReciveTrackState():
    if uartOpenMV.any():
        data = uartOpenMV.read()
        if data:
            try:
                dataStr = data.decode("utf-8").strip()
                print(f"[RX] Recived: {dataStr}")

                if dataStr.startswith("$TRACK:") and dataStr.endswith("#"):
                    stateStr = dataStr[7:-1]
                    return stateStr
                else:
                    print("[RX-ERROR] Invalid message format.")
                    return None
            except Exception as e:
                print(f"[RX-ERROR] Exception during decoding: {e}")
                return None
    return None
