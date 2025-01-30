#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from hcs_ollama_msgs.action import GetVlmResponse
from sensor_msgs.msg import Image

class ClienteDeAccion(Node):

    def __init__(self):
        super().__init__('cliente_de_accion')
        self._accion_cliente = ActionClient(self, GetVlmResponse, '/get_vlm_response')
        self._imagen = None

        # Suscribirse al tópico de la imagen
        self._suscripcion_imagen = self.create_subscription(
            Image,
            '/color/image_raw',
            self.callback_imagen,
            10
        )

        self.received_image = False 

    def callback_imagen(self, msg):
        self._imagen = msg
        self.get_logger().info('Imagen recibida')

    def enviar_meta(self, prompt, validate_stream):
        
        if self._imagen is None:
            self.get_logger().info('No se ha recibido ninguna imagen todavía.')
            return
        
        goal_msg = GetVlmResponse.Goal()
        goal_msg.prompt = prompt
        goal_msg.image = self._imagen
        goal_msg.validate_stream = validate_stream

        self._accion_cliente.wait_for_server()
        self._enviar_goal_future = self._accion_cliente.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._enviar_goal_future.add_done_callback(self.goal_response_callback)

        self.received_image = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('La meta fue rechazada.')
            return

        self.get_logger().info('La meta fue aceptada.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        resultado = future.result().result
        self.get_logger().info('Resultado: {0}'.format(resultado.response))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Respuesta parcial: {0}'.format(feedback.partial_response))

def main(args=None):
    rclpy.init(args=args)
    cliente = ClienteDeAccion()

    prompt = "what objects can see, do a list for me."
    validate_stream = True

    while rclpy.ok() and not cliente.received_image:
        rclpy.spin_once(cliente)
        if cliente._imagen is not None:
            cliente.enviar_meta(prompt, validate_stream)
            break

if __name__ == '__main__':
    main()
