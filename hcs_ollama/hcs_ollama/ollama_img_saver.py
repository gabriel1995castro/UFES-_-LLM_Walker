#!/usr/bin/env python3
# seccion de importe de librerias
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__("Image_saver")

        ## Creacion de subscriptor
        self.subs_img = self.create_subscription(
            Image,
            "/bcr_bot/kinect_camera/image_raw",
            self.img_cb,
            10
        )

        self.br = CvBridge()
        self.received_image = False 

    def img_cb(self, msg: Image):
        # Convertir el mensaje de imagen de ROS2 a formato OpenCV
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return
        
        # Guardar la imagen recibida
        cv2.imwrite('./src/hcs_ollama/assets/imgs/saved_frame.jpg', frame)
        self.get_logger().info('Imagen guardada como saved_frame.jpg')
        self.received_image = True  # Marcar que se recibió una imagen

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaverNode()

    # Mantener el nodo activo hasta recibir una imagen
    while rclpy.ok() and not image_saver.received_image:
        rclpy.spin_once(image_saver)
    
    # Una vez que recibimos la imagen, cerramos el nodo
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

