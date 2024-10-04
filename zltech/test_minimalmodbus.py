import minimalmodbus

instrument = minimalmodbus.Instrument('/dev/ttyUSB2', 1)  # 포트 이름, 슬레이브 주소
instrument.serial.baudrate = 115200  # 설정에 맞게 값 조정

try:
    result = instrument.read_register(0, 1)  # 레지스터 읽기
    print(f"Register value: {result}")
except minimalmodbus.NoResponseError:
    print("No response from the instrument")
