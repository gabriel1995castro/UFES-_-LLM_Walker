import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time  

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'ollama_node', 10)
        self.get_logger().info("Digite 1 para 'true' ou 0 para 'false':")
        self.last = "true"  
        self.last_publish_time = time.time()  

    def run(self):
        try:
            while rclpy.ok():
                user_input = input("> ").strip()  

                if user_input:  
                    msg = String()

                   
                    if user_input == "1":
                        msg.data = "true"
                    elif user_input == "0":
                        msg.data = "false"
                    else:
                        self.get_logger().info("Entrada inválida, digite 1 ou 0.")
                        continue

                    self.last = msg.data  
                    self.publisher_.publish(msg)  
                    self.get_logger().info(f"Enviado: {msg.data}")
                    self.last_publish_time = time.time()  
                else:
                    
                    if time.time() - self.last_publish_time > 5:  
                        msg = String()
                        msg.data = self.last
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Enviado novamente: {msg.data}")
                        self.last_publish_time = time.time()  

                time.sleep(1)  

        except KeyboardInterrupt:
            self.get_logger().info("Nó encerrado pelo usuário.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()