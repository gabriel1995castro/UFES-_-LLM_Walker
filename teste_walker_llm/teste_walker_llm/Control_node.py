import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from random import uniform

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.publisher_ = self.create_publisher(String, 'control_node', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
       
       # Variaveis que simulam valores vindos dos sensores do andador.
        self.g = 9.8
        self.Vatual = uniform (0,5) #gera um valor randomico entre 0 e 5.
        self.Vanterior = 0
        self.Watual = uniform (0,5)
        self.Wanterior = 0
        self.dt = 0.1
        self.Fx = uniform (0,100)
        self.Tz = uniform (0,5)
        self.dv = 90
        self.dw  = 25
        self.mv = 3
        self.mw = 1


    def timer_callback(self):
        #calculo das acelerações:
        aLin = (self.Vatual - self.Vanterior) / self.dt
        aAng = (self.Watual -  self.Wanterior ) / self.dt
        #calculo das velocidades:
        vatual = ((self.Fx * self.g) - (self.mv * aLin))/ self.dv
        watual = ((self.Tz * self.g) - (self.mw * aAng))/ self.dw
        msg = String()
        msg.data = f"v = {vatual:.3f} m/s, w = {watual:.3f} m/s" #cria a frase a ser publicada.
        self.get_logger().info(f"control_node: {msg.data}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ControlNode()  
    rclpy.spin(node)

    node.destroy_node()  
    rclpy.shutdown()

if __name__ == "__main__":
    main()  