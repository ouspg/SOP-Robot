import serial


def main():
    """Read commands from the user and relay them to the Arduino."""

    connection = serial.Serial('/dev/ttyACM0', 115200, timeout=2)

    print('Enter commands as index:value,index:value, for example 0:0,1:-10.')
    print('Values are signed offsets from -90 to 90. Enter quit or exit to stop.')

    try:
        while True:
            command = input('shoulders> ').strip()
            if command.lower() in ('quit', 'exit'):
                break
            if not command:
                continue

            connection.write(f'{command}\n'.encode())
            response = connection.readline().decode(errors='replace').strip()
            if response:
                print(response)
            else:
                print('No response from Arduino.')
    except KeyboardInterrupt:
        print()
    finally:
        connection.close()


if __name__ == '__main__':
    main()
