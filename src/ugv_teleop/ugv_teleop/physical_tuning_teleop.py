# physical_tuning_teleop.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys, select, termios, tty, threading

def map_value(x, in_min, in_max, out_min, out_max):
    """Helper function to map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class PhysicalTuningNode(Node):
    def __init__(self):
        super().__init__('physical_tuning_node')
        self.publisher = self.create_publisher(Int32MultiArray, 'servo_pwm_us', 10)

        # --- YOUR FINAL CALIBRATION DATA ---
        self.servo_calib = [
            {'bw_start': 1434, 'fw_start': 1548}, # 0: Front-Left
            {'bw_start': 1442, 'fw_start': 1554}, # 1: Front-Right
            {'bw_start': 1435, 'fw_start': 1548}, # 2: Back-Left
            {'bw_start': 1432, 'fw_start': 1544}, # 3: Back-Right
        ]
        self.PWM_MIN = 1000.0
        self.PWM_MAX = 2000.0

        # --- INTERACTIVE TUNING PARAMETERS ---
        self.motor_scale_factors = [1.0, 1.0, 1.0, 1.0] 
        self.current_power_level = 0
        self.current_direction = [0, 0, 0, 0] # Default to STOP

        # --- Threading and Terminal Setup ---
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.key_listener_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.print_instructions()
        
    def print_instructions(self):
        self.get_logger().info(
            "\n"
            "--- Physical Tuning Teleop ---\n"
            "Drive Controls:\n"
            "  i : Drive Forward      k : Drive Backward\n"
            "  j : Strafe Left        l : Strafe Right\n"
            "  u : Rotate Left        o : Rotate Right\n"
            "  <space> : EMERGENCY STOP\n"
            "Power Level:\n"
            "  1-9 : Set power to 10%-90%\n"
            "  0   : Set power to 100%\n"
            "Tuning Controls (for straight line driving):\n"
            "  q/a : Tune Front-Left  motor scale (+/- 0.5%)\n"
            "  w/s : Tune Front-Right motor scale (+/- 0.5%)\n"
            "  e/d : Tune Back-Left   motor scale (+/- 0.5%)\n"
            "  r/f : Tune Back-Right  motor scale (+/- 0.5%)\n"
            "  p   : Print current scale factors\n"
            "  ESC : Quit\n"
        )
        
    def key_listener_loop(self):
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == '\x1b': # ESC key
                self.current_power_level = 0
                self.publish_pwm()
                rclpy.shutdown()
                break
            self.handle_key(key)
            self.publish_pwm()

    def handle_key(self, key):
        # --- Kinematics based on your robot's motor orientation ---
        # Forward Motion requires FR and BR to spin 'backwards'
        forward      = [ 1, -1,  1, -1]
        backward     = [-1,  1, -1,  1]
        strafe_left  = [-1, -1,  1,  1]
        strafe_right = [ 1,  1, -1, -1]
        rot_left     = [-1, -1, -1, -1]
        rot_right    = [ 1,  1,  1,  1]

        key = key.lower()
        if key == 'i': self.current_direction = forward
        elif key == 'k': self.current_direction = backward
        elif key == 'j': self.current_direction = strafe_left
        elif key == 'l': self.current_direction = strafe_right
        elif key == 'u': self.current_direction = rot_left
        elif key == 'o': self.current_direction = rot_right
        elif key == ' ':
            self.current_power_level = 0
            self.current_direction = [0, 0, 0, 0]

        # Power Level
        if '1' <= key <= '9': self.current_power_level = int(key) * 10
        elif key == '0': self.current_power_level = 100
        
        # Tuning
        scale_adj = 0.005 # Adjust by 0.5%
        if key == 'q': self.motor_scale_factors[0] += scale_adj
        elif key == 'a': self.motor_scale_factors[0] -= scale_adj
        elif key == 'w': self.motor_scale_factors[1] += scale_adj
        elif key == 's': self.motor_scale_factors[1] -= scale_adj
        elif key == 'e': self.motor_scale_factors[2] += scale_adj
        elif key == 'd': self.motor_scale_factors[2] -= scale_adj
        elif key == 'r': self.motor_scale_factors[3] += scale_adj
        elif key == 'f': self.motor_scale_factors[3] -= scale_adj
        
        if key in 'qawsedrfp':
            self.get_logger().info(f"Scales: {[f'{s:.3f}' for s in self.motor_scale_factors]}")

    def publish_pwm(self):
        motor_pwms = [0.0] * 4
        
        for i in range(4):
            # Step 1 & 2: Apply tuning scale to base power
            scaled_power = self.current_power_level * self.motor_scale_factors[i]
            
            # Step 3: Apply direction for the current movement
            final_power = scaled_power * self.current_direction[i]
            
            # Step 4: Map the final power to the calibrated PWM range
            calib = self.servo_calib[i]
            
            if final_power > 0: # Forward wheel rotation
                motor_pwms[i] = map_value(final_power, 0, 100, calib['fw_start'], self.PWM_MAX)
            elif final_power < 0: # Backward wheel rotation
                motor_pwms[i] = map_value(abs(final_power), 0, 100, calib['bw_start'], self.PWM_MIN)
            else: # Stop
                motor_pwms[i] = (calib['fw_start'] + calib['bw_start']) / 2.0

        msg = Int32MultiArray()
        msg.data = [int(p) for p in motor_pwms] + [90, 90] # Add dummy servo values
        self.publisher.publish(msg)
    
    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalTuningNode()
    rclpy.spin(node) # Will spin until shutdown is called
    node.destroy_node()

if __name__ == '__main__':
    main()