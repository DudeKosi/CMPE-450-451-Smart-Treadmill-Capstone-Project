import serial

def get_speed(speed_dict):
    #serial_port = "/dev/ttyUSB0"
    serial_port = "/dev/ttyS3"
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

    speed_dict[0] = read_data
