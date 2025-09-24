import serial

def get_speed():
    #serial_port = "/dev/ttyUSB0"
    #serial_port = "/dev/ttyS3"
    serial_port = "/dev/ttyS0"
    baud_rate = 9600

    read_data = ''

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)

        if ser.is_open:
            while read_data == '':
                read_data = ser.readline().decode('utf-8').strip()
    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()

    return float(read_data)
