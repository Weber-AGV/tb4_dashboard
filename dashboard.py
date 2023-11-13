from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivymd.uix.progressbar import MDCircularProgress
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class MainWidget(BoxLayout):
    def update_battery_dial(self, battery_level):
        self.ids.battery_dial.value = battery_level

class BatteryApp(App):
    def build(self):
        return MainWidget()

class BatterySubscriber(Node):
    def __init__(self, update_func):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        self.update_func = update_func

    def battery_callback(self, msg):
        battery_level = msg.percentage * 100  # Convert to percentage
        self.update_func(battery_level)

def main(args=None):
    rclpy.init(args=args)
    battery_app = BatteryApp()
    main_widget = battery_app.build()
    battery_subscriber = BatterySubscriber(main_widget.update_battery_dial)
    rclpy.spin(battery_subscriber)

    battery_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
