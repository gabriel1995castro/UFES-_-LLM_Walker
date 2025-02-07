import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class SetFunction(Node):
    def __init__(self):
        super().__init__("set_function")

        #recebe os valores vindos dos nós de controle e do modelo de LLM.
        self.subscription_control = self.create_subscription(String, 'control_node', self.listener_callback, 10)
        self.subscription_llm = self.create_subscription(String, 'ollama_node', self.llm_callback, 10)
        #cria um publisher para sobreescrever o dado do nó de controle.
        self.publisher_control = self.create_publisher(String, 'control_node', 10)


        self.llm_message = None

    def llm_callback(self, msg):

        self.llm_message = msg.data.strip().lower() #recebe um string da llm.
        self.get_logger().info(f"Mensagem do LLM recebida: {self.llm_message}")

    def listener_callback(self, msg):
        
        if self.llm_message is None:
            self.get_logger().info("Aguardando mensagem do LLM.")
            return  

        if self.llm_message == 'true':
            self.get_logger().info("Sem alteração do funcionamento.")
            #Passa o valor original por não ter nenhuma solicitação de alteração.

        elif self.llm_message == 'false':
            
            #apaga pedaços de texto, que vem após o valor de v.
            new_text = re.sub(r',?\s*w\s*=\s*[-+]?\d*\.?\d+\s*m/s', '', msg.data)
            
            output_msg = String()  
            output_msg.data = new_text
            self.publisher_control.publish(output_msg)
            self.get_logger().info("Funcionamento alterado e mensagem publicada.")

def main(args=None):
    rclpy.init(args=args)
    node = SetFunction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()