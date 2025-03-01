import csv
import serial
import sys
import argparse

class CSVReader:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None
        self.reader = None

    def open_file(self):
        """Open the CSV file and prepare it for reading."""
        self.file = open(self.file_path, mode='r', newline='')
        self.reader = csv.reader(self.file)

    def read_next_line(self):
        """Read the next line from the CSV file."""
        if self.reader:
            try:
                return next(self.reader)
            except StopIteration:
                return None  # No more rows to read
        else:
            raise Exception("File not opened. Call open_file() first.")

    def close_file(self):
        """Close the CSV file."""
        if self.file:
            self.file.close()

def send_data_via_serial(port, baudrate, data, delimiter=' '):
    """
    Send an array of data through a serial port.

    Args:
        port (str): The serial port to use (e.g., 'COM3' or '/dev/ttyUSB0').
        baudrate (int): The baud rate for the serial communication.
        data (list): The array of data to send. Elements can be strings, integers, or floats.
        delimiter (str, optional): The delimiter to use between data values. Defaults to a space.
    """
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # Convert each element to string and join them with the delimiter
            data_str = delimiter.join(str(item) for item in data)
            data_str = "rx-ekf-sim " + data_str + "\r\n"
            # Send the data over the serial port
            ser.write(data_str.encode('utf-8'))
            print(f"Sent: {data_str} at timestamp {data[0]}")
    except serial.SerialException as e:
        print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description="Send data from a CSV file through a serial port.")
    parser.add_argument('csv_file', help="Path to the CSV file")
    parser.add_argument('com_port', help="COM port for serial communication")
    parser.add_argument('baud_rate', type=int, help="Baud rate for serial communication")
    args = parser.parse_args()

    # Create CSVReader instance
    csv_reader = CSVReader(args.csv_file)
    csv_reader.open_file()

    # Read and print the first row
    first_row = csv_reader.read_next_line()
    if first_row is not None:
        print("First row:", first_row)

    # Read each row and send data through the serial port
    line = csv_reader.read_next_line()
    while line is not None:
        send_data_via_serial(args.com_port, args.baud_rate, line)
        line = csv_reader.read_next_line()

    csv_reader.close_file()

if __name__ == '__main__':
    main()
