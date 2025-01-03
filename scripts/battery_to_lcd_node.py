import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState  # Import for /battery_status
from std_msgs.msg import String  # Import for /lcd

class BatteryToLCD(Node):
    def __init__(self):
        super().__init__('battery_to_lcd_node')

        # Subscriber to /battery_status
        self.battery_subscription = self.create_subscription(
            BatteryState,  # Message type for /battery_status
            '/battery_status',  # Topic name
            self.battery_status_callback,
            10  # QoS
        )
        self.battery_subscription  # To avoid unused variable warning

        # Publisher to /lcd_display/row1
        self.lcd_publisher = self.create_publisher(
            String,  # Message type for /lcd
            '/lcd_display/row1',  # Topic name
            10  # QoS
        )
        
        self.get_logger().info("Battery to LCD node is up and running.")

    def battery_status_callback(self, msg):
        # Extract relevant information from BatteryState message
      #  battery_percentage = msg.percentage * 100 if msg.percentage >= 0 else "Unknown"
        voltage = msg.voltage
       # temperature = msg.temperature if not msg.temperature == float('nan') else "N/A"

        # Format the information for the LCD as a string
        lcd_message = String()
        lcd_message.data = (
        #    f"Battery: {percentage:.2f}%\n"
            f"Voltage: {voltage:.2f}V\n"
        #    f"Temperature: {temperature:.2f}°C"
        )

        # Publish to /lcd_display/row1
        self.lcd_publisher.publish(lcd_message)
        self.get_logger().info(f"Published to /lcd_display/row1:\n{lcd_message.data}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryToLCD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()