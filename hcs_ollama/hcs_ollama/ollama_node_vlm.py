#!/usr/bin/env python3
# Seccion de importe de librerias
# Lirberias de interaccion con ros
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
# Librerias de interaccion con action 
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
# Librerias propias
from hcs_ollama.utils import ollama_functions as oll_f
# Importe de librerias de interfaz de ros
from hcs_ollama_msgs.action import GetVlmResponse
# Librerias de comunicacion de ia
import ollama
# Importe de librerias utilitarias
from time import time as t_count
from cv_bridge import CvBridge
import cv2



## TODO: Crear funcciones necesarioas para cumplir con las necesidades de u 
## 2. Sistema de validacion continua de estado de conexion con llm

# Creacion de clase
class OllavaHcsNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("ollama_communication_node")

        # variables de instancia de nodo
        self.cvBridge = CvBridge()
        # self.received_image:bool = False
        # En este caso seria importante agregar las configuraciones de seleccion de agentes para las cuales se crea el nodo
        # Seccion de declaracion de parametros para interaccion de nodo
        self.declare_parameter("ai_model", "llava:13b")
        self.declare_parameter("temperature", 0.6)
        self.declare_parameter("max_tokens_output", 532)
        self.declare_parameter("seed", -1)
        self.declare_parameter("path_img", "./src/hcs_ollama/assets/imgs/saved_frame.jpg")

        self.get_logger().info("Nodo de ejecucion de ollama creado")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de configuracion de nodo para comunicacion con ollama")

        # Asignacion e inicializacion de variables de instancia desde parametros ingresados
        self.model_ai:str = self.get_parameter("ai_model").get_parameter_value().string_value
        self.temperature_value:float = float(self.get_parameter("temperature").get_parameter_value().double_value)
        self.num_predict:int = self.get_parameter("max_tokens_output").get_parameter_value().integer_value
        self.seed_:int = self.get_parameter("seed").get_parameter_value().integer_value
        self.path_img_:str = self.get_parameter("path_img").get_parameter_value().string_value

        # Validacion de existencia de modelo 
        if not oll_f.is_model_available_locally(self.model_ai):
            self.get_logger().error(f"Error, modelo de llama '{self.model_ai}' no encontrado en la base local.")
            return TransitionCallbackReturn.FAILURE

        # Variables de instancia configuradas
        self.goal_handle = None

        self.get_logger().info("Nodo configurado correctamente")
        # Seccion para creacion de publishers en caso de ser necesario
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de activacion de nodo")

        # Creacion de action para solicitud de llm
        try:
            self.prompt_action_ = ActionServer(
                self,
                GetVlmResponse,
                'get_vlm_response',
                #self.llm_execute_cb,
                execute_callback = self.llm_execute_cb,
                goal_callback = self.goal_llm_callback,
                handle_accepted_callback = self.accepted_llm_callback,
                cancel_callback = self.cancel_llm_callback
            )
        except:
            return TransitionCallbackReturn.FAILURE
        #print(3)
        self.get_logger().info("Nodo activado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de desactivacion de nodo")

        if self.prompt_action_:
            self.prompt_action_.destroy()

        self.get_logger().info("Nodo desactivado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de limpieza de variables configuradas en el nodo")
        
        self.model_ai = ""
        self.temperature_value = -1.0
        self.num_predict = -1     
        self.path_img_ = ""

        self.get_logger().info("Nodo limpiado correctamente")
        return TransitionCallbackReturn.SUCCESS
        
    #### SOLICITUD DE PROMPT DE USUARIO
    def goal_llm_callback(self, goal_request):
        self.get_logger().info('Received goal request the next sentence: {0}'.format(goal_request.prompt))
        return GoalResponse.ACCEPT
    
    def accepted_llm_callback(self, goal_handle):
        self.get_logger().info('Goal accepted and starting execution.')

        # Inicializar hilo de ejecucion
        return goal_handle.execute()
    
    def cancel_llm_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal.')
        return CancelResponse.ACCEPT
    
    def llm_execute_cb(self, goal_handle):
        # Seccion de almacenamiento de imagen
        self.get_logger().info('Executing goal...')

        feedback_msg = GetVlmResponse.Feedback()
        feedback_msg.partial_response = ''

        instruction_:str = goal_handle.request.prompt
        # Primera seccion almacenamiento de imagen
        try:
            start_img_time = t_count()
            temp_frame = self.cvBridge.imgmsg_to_cv2(goal_handle.request.image, desired_encoding='bgr8')

            # Almacenamiento de imagen recibida
            cv2.imwrite(self.path_img_, temp_frame)

            end_img_time = t_count()
            duration_img = end_img_time - start_img_time
            feedback_msg.partial_response = "Imagen almacenada con exito."
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Image save completed in {0:.2f} seconds.'.format(duration_img))
        except Exception as e:
            self.get_logger().error(f"An error occurred during save image: {e}")
            goal_handle.abort()
            result = GetVlmResponse.Result()
            result.response = ''
            return result
        
        # Validacion de tiempo de ejecucion
        start_process_img = t_count()
        # Procesamiento de imagen
        result_text = ollama.generate(
            model = self.model_ai,
            prompt = instruction_,
            images = [self.path_img_],
            stream = False
        )["response"]
        self.get_logger().info(result_text)
        feedback_msg.partial_response = result_text
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        
        result = GetVlmResponse.Result()
        result.response = result_text
        end_process_img = t_count()
        duration_process_img = end_process_img - start_process_img
        self.get_logger().info('Goal completed in {0:.2f} seconds.'.format(duration_process_img))

        return result
# Funcion main de ejecucion de nodo
def main():
    rclpy.init()
    # Llamado de nodo
    node = OllavaHcsNode()
    # configuracion y activacion automatica de nodo
    node.trigger_configure()
    node.trigger_activate()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()