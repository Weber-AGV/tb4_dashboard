from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
import rclpy
from rclpy.node import Node


class MainWidget(BoxLayout):
    def update_battery_dial(self, battery_level):
        self.ids.battery_dial.value = battery_level

class BatteryApp(App):
    def build(self):
        return BoxLayout()

    def on_start(self):
        rclpy.init()
        self.node = BatteryNode()
        # Update interval set to 1/60 seconds (or as needed)
        Clock.schedule_interval(self.update_ros2, 1.0 / 60.0)

    def update_ros2(self, dt):
        rclpy.spin_once(self.node, timeout_sec=0)

    def on_stop(self):
        rclpy.shutdown()


class BatteryNode(Node):
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

def main():
    app = BatteryApp()
    app.run()

if __name__ == '__main__':
    main()
