import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial
import logging

class NemaSerialReader(Node):

    def __init__(self):
        super().__init__('nema_serial_reader')
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.publisher_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.serial_port = '/dev/ttyACM0'  # Update this with your port
        self.serial_baud = 9600  # Common baud rate for GNSS devices
        self.serial_connection = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
        self.get_logger().info(f"Connected to {self.serial_port} at {self.serial_baud} baud.")
        self.create_timer(0.1, self.read_from_serial)

    def read_from_serial(self):
        if self.serial_connection.in_waiting > 0:
            line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
            # logging.info(line)
            if '$GPGSV' in line:
                self.log_gpgsv_info(line)
            
            if '$GPGGA' in line:  # Check for GPGGA string
                logging.debug(line)
                parts = line.split(',')

                if parts[2] == '':
                    self.get_logger().warn("No GPS fix detected.")
                    return
                
                navsatfix_msg = self.convert_nema_to_navsatfix(line)
                logging.info("Sat Fix: Lat: %f, Lon: %f, Alt: %f", navsatfix_msg.latitude, navsatfix_msg.longitude, navsatfix_msg.altitude)
                self.publisher_.publish(navsatfix_msg)

    def convert_nema_to_navsatfix(self, nema_string):
        parts = nema_string.split(',')

        # Parse latitude and longitude
        lat = self.convert_to_degrees(parts[2], parts[3])
        lon = self.convert_to_degrees(parts[4], parts[5])
        altitude = float(parts[9])

        # Create NavSatFix message
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = Header()
        navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
        navsatfix_msg.header.frame_id = "gps_frame"

        navsatfix_msg.status.status = int(parts[6])
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS

        navsatfix_msg.latitude = lat
        navsatfix_msg.longitude = lon
        navsatfix_msg.altitude = altitude

        navsatfix_msg.position_covariance = [0.0] * 9
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        return navsatfix_msg

    def convert_to_degrees(self, raw_value, direction):
        degrees = int(float(raw_value) / 100)
        minutes = float(raw_value) - degrees * 100
        decimal_degrees = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        return decimal_degrees
    
    def log_gpgsv_info(self, nmea_sentence):
        if not nmea_sentence.startswith("$GPGSV"):
            logging.warning("The provided sentence is not a GPGSV message.")
            return

        # Split the NMEA sentence by commas
        parts = nmea_sentence.split(',')

        # Ensure the sentence has enough fields
        if len(parts) < 8:
            logging.error("Incomplete GPGSV sentence.")
            return

        # Extract GPGSV information
        num_of_sentences = int(parts[1])
        sentence_num = int(parts[2])
        num_of_sats_in_view = int(parts[3])

        logging.debug(f"GPGSV Info: Total Sentences: {num_of_sentences}, Sentence Number: {sentence_num}, Satellites in View: {num_of_sats_in_view}")

        # Process satellite data (each satellite is described by 4 fields: PRN, Elevation, Azimuth, SNR)
        sat_info_start_idx = 4
        for i in range(sat_info_start_idx, len(parts) - 4, 4):
            try:
                prn = parts[i]
                elevation = parts[i + 1]
                azimuth = parts[i + 2]
                snr = parts[i + 3] if parts[i + 3] else 'N/A'  # SNR might be empty
                logging.debug(f"Satellite PRN: {prn}, Elevation: {elevation}°, Azimuth: {azimuth}°, SNR: {snr} dB")
            except IndexError:
                logging.error(f"Error parsing satellite info at index {i}.")
                break

def main(args=None):
    rclpy.init(args=args)
    node = NemaSerialReader()
    rclpy.spin(node)
    node.serial_connection.close()  # Close the serial connection on shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
