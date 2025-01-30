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
from hcs_ollama.utils import langchain_owm_functions as lng_func
# Importe de librerias de interfaz de ros
from hcs_ollama_msgs.action import GetLlmResponse
from hcs_ollama_msgs.msg import SystemConfigurationAi
from hcs_ollama_msgs.srv import Tokenize
from hcs_ollama_msgs.srv import Detokenize
from hcs_ollama_msgs.srv import Embedding
# Librerias de comunicacion de ia
from langchain_ollama import ChatOllama
from langchain_ollama import OllamaEmbeddings
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain.callbacks.manager import CallbackManager
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import SystemMessagePromptTemplate
from langchain_core.prompts import HumanMessagePromptTemplate
# Librerias para monitore de tokens
from langchain_community.callbacks.manager import get_openai_callback
# from langchain.callbacks import get_openai_callback
# Libreria para codificacion de tokens por cada palabra ingresada
from tiktoken import get_encoding
# Librerias para validacion de tipo de datos 
from typing import Any
# Importe de librerias utilitarias
from time import time as t_count

## TODO: Crear funcciones necesarioas para cumplir con las necesidades de u 
## 2. Sistema de validacion continua de estado de conexion con llm

# Creacion de clase
class OllamaHcsNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("ollama_communication_node")

        # variables de instancia de nodo
        self.model_connection = None
        # En este caso seria importante agregar las configuraciones de seleccion de agentes para las cuales se crea el nodo
        # Seccion de declaracion de parametros para interaccion de nodo
        self.declare_parameter("ai_model", "llama3.1")
        self.declare_parameter("temperature", 0.6)
        self.declare_parameter("max_tokens_output", 216)
        self.declare_parameter("seed", -1)
        self.declare_parameter("tokenizer_model", "cl100k_base")
        self.declare_parameter("activate_embedding", True)

        self.get_logger().info("Nodo de ejecucion de ollama creado")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de configuracion de nodo para comunicacion con ollama")

        # Asignacion e inicializacion de variables de instancia desde parametros ingresados
        self.model_ai:str = self.get_parameter("ai_model").get_parameter_value().string_value
        self.temperature_value:float = float(self.get_parameter("temperature").get_parameter_value().double_value)
        self.num_predict:int = self.get_parameter("max_tokens_output").get_parameter_value().integer_value
        self.seed_:int = self.get_parameter("seed").get_parameter_value().integer_value
        self.tokenizer_model:str = self.get_parameter("tokenizer_model").get_parameter_value().string_value
        self.bool_embedding_:bool = self.get_parameter("activate_embedding").get_parameter_value().bool_value

        # Conexion a tokenizador 
        self.tiktoken_ = get_encoding(self.tokenizer_model)

        # Validacion de existencia de modelo 
        if not oll_f.is_model_available_locally(self.model_ai):
            self.get_logger().error(f"Error, modelo de llama '{self.model_ai}' no encontrado en la base local.")
            return TransitionCallbackReturn.FAILURE

        # Variables de instancia configuradas
        self.goal_handle = None
        
        # Servicio de tokenizacion y detokenizacion
        self.tokenize_service_ = self.create_service(
            Tokenize,
            "tokenize",
            self.cb_token
        )
        # Servicio de detokenizacion
        self.detokenize_service_ = self.create_service(
            Detokenize,
            "detokenize",
            self.cb_detoken
        )

        # validacion de uso de modelo embedding y creacion de servicio que permite la conversion de los prompts
        if self.bool_embedding_:
            # Creacion de modelo para embedding
            self.model_embedding = OllamaEmbeddings(
                model = self.model_ai
            )

            # Creacion de servicio
            self.embedding_service_ = self.create_service(
                Embedding,
                "embedding_input",
                self.cb_embedding
            )

        self.get_logger().info("Nodo configurado correctamente")
        # Seccion para creacion de publishers en caso de ser necesario
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de activacion de nodo")

        # Configuracion de callback manager para optimizacion de proceso    
        callback_manager = CallbackManager([StreamingStdOutCallbackHandler])
        #print(1)
        # Conexion con modelo de ollama
        self.model_connection = ChatOllama(
            model = self.model_ai,
            temperature = self.temperature_value,
            num_predict = self.num_predict,
            stop = None,
            seed = self.seed_,
            callback_manager = callback_manager,
            verbose = True 
        )
        #print(2)
        # Creacion de action para solicitud de llm
        try:
            self.prompt_action_ = ActionServer(
                self,
                GetLlmResponse,
                'get_llm_response',
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

        del self.model_connection
        
        self.get_logger().info("Nodo desactivado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Inicio de limpieza de variables configuradas en el nodo")
        
        self.model_ai:str = ""
        self.temperature_value:float = -1.0
        self.num_predict:int = -1            
        
        # Limpieza de servicios creados en la configuracion
        self.destroy_service(self.tokenize_service_)
        self.destroy_service(self.detokenize_service_)

        if self.bool_embedding_:
            del self.model_embedding
            self.destroy_service(self.embedding_service_)

        self.bool_embedding_:bool = False

        self.get_logger().info("Nodo limpiado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    # Seccion de creacion de callbacks de servicios
    # Servicio de tokenizacion
    def cb_token(self, request, response) -> Any:
        txt_to_tokenize = str(request.text)
        try:
            response.tokens = self.tiktoken_.encode(txt_to_tokenize)
        except Exception as e:
            self.get_logger().error(f"Error en la tokenizacion del texto ingresado. {e}")
            response.tokens = []
        
        return response
    
    # Servicio de detokenizacion
    def cb_detoken(self, request, response) -> Any:
        list_to_text = list(request.tokens)
        try:
            response.text = self.tiktoken_.decode(list_to_text)
        except Exception as e:
            self.get_logger().error(f"Error en la detokenizacion de la lista ingresada. {e}")
            response.text = ""
        
        return response

    # Servicio para embeddings
    def cb_embedding(self, request, response) -> Any:
        input_text = str(request.input)

        try:
            response.vector = self.model_embedding.embed_query(input_text)
        except Exception as e:
            self.get_logger().error(f"Error en la conversion del texto agregado a vector. {e}")
            response.vector = []
        
        return response
        
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
        # Segunda prueba de ejecuion de procesamiento
        # Llamado de variables de scope local para ejecuccion de proceso
        prompt_request: str = goal_handle.request.prompt
        system_config: SystemConfigurationAi = goal_handle.request.ai_config
        validate_stream: bool = goal_handle.request.validate_stream
        try:
            self.get_logger().info('Executing goal...')
            # Validacion de tiempo
            start_time = t_count()
            feedback_msg = GetLlmResponse.Feedback()
            feedback_msg.partial_response = ''

            complete_msg = ""
            system_desc = "You are a helpful {first_role}. Your name is {name_robot}."
            
            if system_config.system_prompt != "":
                system_desc += " {system_prompt}."
            else:
                system_desc+="{system_prompt}"

            if system_config.aditional_config != "":
                system_desc += " {aditional_config}."
            else:
                system_desc += "{aditional_config}"

            if system_config.use_language_val:
                system_desc += " Upon receiving the request in {input_language}, your response must be provided in {output_language}. DO NOT INCLUDE THE TRANSLATION IN YOUR RESPONSE."
            else:
                system_desc += "{input_language}{output_language}"
                
            chat_prompt_template = ChatPromptTemplate.from_messages([
                # Seccion de descripcion de sistema}
                SystemMessagePromptTemplate.from_template(system_desc),
                # Seccion de descripcion de usuario
                HumanMessagePromptTemplate.from_template("{user_input}")])
            

            messages = chat_prompt_template.format_messages(
                first_role = system_config.first_role,
                name_robot = system_config.name, 
                system_prompt = system_config.system_prompt,
                aditional_config = system_config.aditional_config,
                input_language = system_config.input_language,
                output_language = system_config.output_language,
                user_input = prompt_request
            )

            feedback_interval = 5  # Cantidad de intervalos para la publicacion de chunks
            chunk_counter = 0

            tools = [
                lng_func.MoveToPose,
                lng_func.RelativeMove,
                lng_func.RejectRequest
            ]

            if system_config.use_tools:
                tools_model = self.model_connection.bind_tools(tools)
            
            with get_openai_callback() as cb_token_monitoring:
                if validate_stream:
                    self.get_logger().info("Streaming is in progress")
                    for chunk in self.model_connection.stream(messages):
                        # Verifica si se ha solicitado la cancelación
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            self.get_logger().info('Goal canceled.')
                            return

                        complete_msg += chunk.content
                        chunk_counter += 1
                        #print(chunk_counter)

                        # Controlar la frecuencia de publicación de feedback
                        if chunk_counter % feedback_interval == 0:
                            feedback_msg.partial_response = complete_msg
                            goal_handle.publish_feedback(feedback_msg)
                            self.get_logger().debug('Feedback published.')
                
                # Validacion de mensaje completo
                else:
                    self.get_logger().info("Streaming is not in use.")
                    msg_ = self.model_connection.invoke(messages)
                    complete_msg = msg_.content
                
                print(msg_)

                self.get_logger().info(f"Total tokens utilized in this: {cb_token_monitoring.total_tokens}")
                self.get_logger().info(f"Tokens consumed for prompt: {cb_token_monitoring.prompt_tokens}")
            
            feedback_msg.partial_response = complete_msg
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug('Final feedback published.')
            # actualizar estado completado de objetivo
            goal_handle.succeed()
            # Obtener resultado final
            result = GetLlmResponse.Result()
            result.response = complete_msg
            # Log de tiempo de ejecucion de proceso
            end_time = t_count()
            duration = end_time - start_time
            self.get_logger().info('Goal completed in {0:.2f} seconds.'.format(duration))

            return result

        except Exception as e:
            self.get_logger().error(f"An error occurred during streaming: {e}")
            goal_handle.abort()
            result = GetLlmResponse.Result()
            result.response = ''
            return result

# Funcion main de ejecucion de nodo
def main():
    rclpy.init()
    # Llamado de nodo
    node = OllamaHcsNode()
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