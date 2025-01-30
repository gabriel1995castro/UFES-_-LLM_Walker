#!/usr/bin/env python3
# El siguiente nodo comprende la logica de llamado de solicitudes para un multiagente
# Version 0.1.0
# Seccion de librerias de configuracion de nodo
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
# Librerias para configuracion de action
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
# Librerias para la configuracion de modelo
from langchain_ollama import ChatOllama
from langchain.callbacks.manager import CallbackManager
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
# from langchain.document_loaders import PDFPlumberLoader # Deprecado   
from langchain_community.document_loaders import PDFPlumberLoader
# Librerias para configuracion de mensajes de langchain
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import SystemMessagePromptTemplate
from langchain_core.prompts import HumanMessagePromptTemplate
# Librerias para manejo de grafos de ejecucion
from langgraph.graph import StateGraph
from langgraph.graph import END
# Librerias de interfaces para manejo action, servicios y msg de ros2
from hcs_ollama_msgs.action import GetMultiAgentResponse
from hcs_ollama_msgs.msg import EvaluatorResponse
from hcs_ollama_msgs.msg import ExecutorResponse
from hcs_ollama_msgs.msg import LocalizatorResponse
# Seccion de librerias propias
from hcs_ollama.utils import ollama_functions as oll_f
from hcs_ollama.utils import structured_output
# Librerias para manejo de tipado de variables
from typing import TypedDict
from typing import Literal
from typing import List
from typing import Any
# Seccion de librerias utilitarias
from time import time as t_count
import yaml
import json

# Clase de ejecucion de agente
class AgentState(TypedDict):
    request_state: str
    response_dict: dict

# Creacion de clase de nodo
class multiagentOllamaNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("ollama_multiagent_node")
        # Variable de instancia para inicializacion de modelo
        self.model_connection = None

        # Declaracion de parametros iniciales
        self.declare_parameter("ai_model", "llama3.1")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("system_config_path", "./src/hcs_ollama/config/movemtent_config.yaml")
        self.declare_parameter("map_description", "./src/hcs_ollama/config/map_description.json")
        self.declare_parameter("map_semantic_description", "./src/hcs_ollama/config/description_general.pdf")
        # Validacion de estado inicial
        self.get_logger().info("Inicializacion de nodo de action para multiagentes")

    # Primer estado, configuracion de nodo
    def on_configure(self, state: LifecycleState)-> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        # Inicializacion de variables de intancia para los parametros generales
        self.model_ai: str = self.get_parameter("ai_model").get_parameter_value().string_value
        self.temperature_value:float = float(self.get_parameter("temperature").get_parameter_value().double_value)
        # Lectura de configuracion de agentes
        try:
            path_temp = self.get_parameter("system_config_path").get_parameter_value().string_value
            with open(path_temp) as stream:
                self.system_config = yaml.safe_load(stream)
        except Exception as e:
            self.system_config = None
            self.get_logger().error(f"Error en la lectura de configuracion. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Lectura de descripcion de mapa
        try:
            path_map_temp = self.get_parameter("map_description").get_parameter_value().string_value
            with open(path_map_temp, 'r', encoding = 'utf-8') as file:
                self.map_config = json.load(file) 
                self.map_config = self.map_config["area_layout"]["offices"]
        except Exception as e:
            self.map_config = None
            self.get_logger().error(f"Error en la lectura del archivo de descripcion del mapa. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Lectura de archivos pdf
        try:
            path_doc: str = self.get_parameter("map_semantic_description").get_parameter_value().string_value
            loader_pdf = PDFPlumberLoader(path_doc)
            documents: List[Any] = loader_pdf.load()

            # Conversion a texto general
            self.text_map = ""
            for doc in documents:
                self.text_map+=doc.page_content
                self.text_map+="\n"

        except Exception as e:
            self.text_map = ""
            self.get_logger().error(f"Error en la lectura del archivo de descripcion semantica del mapa. {e}")
            return TransitionCallbackReturn.FAILURE

        # Variable de instancia para seguimiento de objetivo
        self.goal_handle = None

        # Validacion de existencia de nodo usado
        if not oll_f.is_model_available_locally(self.model_ai):
            self.get_logger().error(f"Error, modelo de llama '{self.model_ai}' no encontrado en la base local.")
            return TransitionCallbackReturn.FAILURE
        
        # Validacion de transicion de estado
        self.get_logger().info("Nodo configurado correctamente")
        # Seccion para creacion de publishers en caso de ser necesario
        return TransitionCallbackReturn.SUCCESS
    
    # Proceso de activacion de nodo
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        # Configuracion de conexion de modelo
        # Creacion de callback manager, (Recomendacion por literatura para mejorar el tiempo de ejecicion)
        callback_manager = CallbackManager([StreamingStdOutCallbackHandler])
        self.get_logger().info("Inicio de configuracion de conexion con ollama")
        try:
        # Asignacion de modelo
            self.model_connection = ChatOllama(
                model = self.model_ai,
                temperature = self.temperature_value,
                callback_manager = callback_manager
            )
        except:
            print("Error en la configuracion del modelo en general")

        self.get_logger().info("Inicio de configuracion de agente evaluador")
        try:
            # Configuracion de agentes usando instrucciones creadas
            self.evaluator_agent = self.config_agent(
                self.model_connection,
                self.system_config["config_agents"]["evaluator_agent"],
                self.system_config["config_user"]["evaluator_user"],
                structured_output.EvaluatorResponseStructure
            )
        except Exception as e:
            print(f"Error en configuracion de agente evaluador. {e}")

        self.get_logger().info("Inicio de configuracion de agente localizador")
        try:
            # Cofiguracion de agente localizador 
            self.localizator_agent = self.config_agent(
                self.model_connection,
                self.system_config["config_agents"]["localizator_agent"],
                self.system_config["config_user"]["localizator_user"],
                structured_output.LocalizatorResponseStructure
            )
        except:
            print("Error en configuracion de agente localizador")
        
        # print(self.text_map)
        # Creacion de ciclo de ejecucion
        # Primero ejecucion de nodop
        def get_actual_request(state: AgentState) -> AgentState:
            return state
        
        def validator_request(state: AgentState) -> AgentState:
            return state
        
        def localizator_request(state: AgentState) -> AgentState:
            context_value = self.text_map
            location_search = state["response_dict"]["Evaluation"].location_objective
            result = self.localizator_agent.invoke(
                {
                    "location": location_search,
                    "context": context_value
                }
            )
            state["response_dict"]["Localization"] = result
            return state
        
        def executor_request(state: AgentState) -> AgentState:
            # print("Proceso de ejecucion")
            objective_localization = state["response_dict"]
            # print(objective_localization)
            execution_order = self.get_center_position(objective_localization)
            if execution_order is not None:
                var_temp = structured_output.ExecutorResponseStructure(coordinates = execution_order, actual_state = 'waiting')
                # if execution_order is not None:
                state["response_dict"]["Execution"] = var_temp
            else:
                state["response_dict"]["Execution"] = execution_order

            return state
        
        def publisher_request(state: AgentState) -> AgentState:
            return state
        
        def evaluator_router(state: AgentState) -> Literal["validator", "not_relevant"]:
            request = state["request_state"]
            result = self.evaluator_agent.invoke({
                request
            })
            
            state["response_dict"]["Evaluation"] = result
            
            if result.is_movement_request == "Yes":
                return "validator"
            else:
                return "not_relevant"

        def validator_router(
            state: AgentState,
        ) -> Literal["localizator", "executor", "publisher"]:

            validate = state["response_dict"]
            lista_llaves = list(validate.keys())

            if 'Localization' not in lista_llaves:
                return "localizator"

            if 'Execution' not in lista_llaves:
                return 'executor'
            
            return "publisher"
        
        # Creacion y compilacion de multiagentes
        workflow = StateGraph(AgentState)

        workflow.add_node("evaluator", get_actual_request)
        workflow.add_node("validator", validator_request)
        workflow.add_node("localizator", localizator_request)
        workflow.add_node("executor", executor_request)
        workflow.add_node("publisher", publisher_request)

        workflow.set_entry_point("evaluator")
        workflow.add_conditional_edges(
            "evaluator", evaluator_router, {"validator": "validator", "not_relevant": END}
        )
        workflow.add_conditional_edges(
            "validator", validator_router, 
            {
                "localizator": "localizator",
                "executor": "executor",
                "publisher": "publisher"
            }
        )
        workflow.add_edge("localizator", "validator")
        workflow.add_edge("executor", "validator")
        workflow.add_edge("publisher", END)
        # Configuracion final de modelo personalizado
        self.multi_model = workflow.compile()
        # self.multi_model.
        # Creacion de action para comuniacion de solicitudes de ros
        try:
            self.multiagent_action = ActionServer(
                self,
                GetMultiAgentResponse,
                'get_multiagent_response',
                execute_callback = self.llm_execute_cb,
                goal_callback = self.goal_llm_callback,
                handle_accepted_callback = self.accepted_llm_callback,
                cancel_callback = self.cancel_llm_callback
            )
        except Exception as e:
            self.get_logger().error(f"Error en la transicion de activacion. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Validacion de transicion de activacion
        self.get_logger().info("Nodo activado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    # Configuracion de deactivacion de nodo
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        del self.evaluator_agent
        del self.localizator_agent

        if self.multiagent_action:
            self.multiagent_action.destroy()

        del self.model_connection
        
        self.get_logger().info("Nodo desactivado correctamente")
        return TransitionCallbackReturn.SUCCESS

    # Configuracion de limpieza de nodo
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")
        
        self.model_ai:str = ""
        self.temperature_value:float = -1.0
        self.system_config = None
        self.map_config = None
        self.text_map = ""

        self.get_logger().info("Nodo limpiado correctamente")
        return TransitionCallbackReturn.SUCCESS
    
    # Seccion de callbacks para interaccion de action
    # Callbacl de recepcion de solicitud
    def goal_llm_callback(self, goal_request):
        self.get_logger().info('Request received successfully.\n Sending for processing:\n {0}'.format(goal_request.prompt))
        return GoalResponse.ACCEPT
    
    # Callback de respuesta a de solicitud aceptada
    def accepted_llm_callback(self, goal_handle):
        self.get_logger().info('Goal accepted and starting execution.')
        # Inicializar hilo de ejecucion
        return goal_handle.execute()
    
    # Callback para cancelacion de procesamiento de solicitud
    def cancel_llm_callback(self, goal_handle):
        self.get_logger().info('Request to cancel received successfully.')
        return CancelResponse.ACCEPT
    
    # Funcion para ejecucion general de action
    def llm_execute_cb(self, goal_handle):
        # Llamado de variables de instancia para ejecucion de callback
        local_prompt_request : str = str(goal_handle.request.prompt)

        # Validacion de ejecucion de primera seccion
        try:
            self.get_logger().info("Prueba de ejecucion de modelo de agentes multiples.")

            start_time = t_count()
            # Creacion de mensaje de feedback
            feedback_msg = GetMultiAgentResponse.Feedback()

            # Variables de respuesta parcial
            feedback_msg.status = "processing"

            self.get_logger().info("Inicio de invocacion de multiagentes")
            # Inicializacion de primer estado de ejecucion
            initial_state_ = {"request_state": local_prompt_request, "response_dict": dict()}

            final_response:dict = dict()
            for event in self.multi_model.stream(initial_state_, stream_mode = "updates"):
                # Validacion de estado de procesamiento
                try:
                    feedback_msg.agent_name = next(iter(event)).capitalize()
                    feedback_msg.feedback_response = str(event[list(event.keys())[0]]['response_dict'])
                    # final_response = event[feedback_msg.agent_name]['response_dict']
                    final_response = event[list(event.keys())[0]]['response_dict']
                    self.get_logger().info(f"Feedback Publicado.")
                    self.get_logger().info(f"Estatus de solicitud: {feedback_msg.status}")
                    self.get_logger().info(f"Agente en procesamiento: {feedback_msg.agent_name}")
                    self.get_logger().info(f"Mensaje actual: {feedback_msg.feedback_response}")
                    
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().debug('Feedback published.')
                except Exception as e:
                    self.get_logger().error(f"Error en solicitud de ejecucion en stream. {e}")
                
        except Exception as e:
            self.get_logger().error(f"An error occurred during streaming: {e}")
            goal_handle.abort()
            result = GetMultiAgentResponse.Result()
            # result.response = ''
            return result
        
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().debug('Final feedback published.')
        # actualizar estado completado de objetivo
        goal_handle.succeed()
        # Obtener resultado final
        result = GetMultiAgentResponse.Result()

        evaluator_resp = EvaluatorResponse()
        # Asignacion de valores de respuesta
        evaluation_response_from_final = final_response['Evaluation']
        # Asignaciondes de valores internos
        try:
            evaluator_resp.validation_request = evaluation_response_from_final.is_movement_request
        except Exception as e:
            pass
        
        if evaluator_resp.validation_request == "Yes":
            evaluator_resp.translate_request = evaluation_response_from_final.translate_task
            evaluator_resp.objective = evaluation_response_from_final.location_objective
        
        localizator_resp = LocalizatorResponse()
        try:
            localization_response_from_final = final_response['Localization']
            # Asignacion de valores internos
            if localization_response_from_final.known_info:
                localizator_resp.existing_info = localization_response_from_final.known_info
                localizator_resp.location_name = localization_response_from_final.location_name
        except Exception as e:
            pass

        executor_resp = ExecutorResponse()
        try:
            executor_response_from_final = final_response['Execution']
            if localization_response_from_final.known_info:
                executor_resp.x = executor_response_from_final.coordinates['x']
                executor_resp.y = executor_response_from_final.coordinates['y']
                # executor_resp.theta = localization_response_from_final.coordinates['x']
                executor_resp.status_execution = executor_response_from_final.actual_state
        except Exception as e:
           pass

        result.evaluator_response = evaluator_resp
        result.localizator_response = localizator_resp
        result.executor_response = executor_resp
        # Log de tiempo de ejecucion de proceso
        end_time = t_count()
        duration = end_time - start_time
        self.get_logger().info('Goal completed in {0:.2f} seconds.'.format(duration))

        return result

    # Funcion generalizada para la configuracion de un agente
    def config_agent(self, llm_connection, system_config:str, user_config:str, structured_output = None):

        # Creacion de mensaje de sistema 
        system_prompt = SystemMessagePromptTemplate.from_template(system_config)
        user_prompt = HumanMessagePromptTemplate.from_template(user_config)

        # Configuracion de mensaje general
        chat_msg = ChatPromptTemplate.from_messages([
            system_prompt,
            user_prompt
        ])

        # Validacion de respuesta estructurada
        if structured_output is not None:
            modelo_structured = llm_connection.with_structured_output(structured_output)
        
            model = chat_msg | modelo_structured
            return model
        
        model = chat_msg | llm_connection
        return model
    
    # Funcion de clase para obtencion de centro de posicion de un valor especifico
    def get_center_position(self, existing_area: dict):
        desct_office = None

        try:
            for value_ in self.map_config:
                name_temp = value_.get('name')
                name_temp = name_temp.replace('-', ' ').replace('(HCS)', '').upper()
                
                if existing_area.get('Localization').location_name.upper() in name_temp:
                    desct_office = value_.get('central_point').get('coordinates')
                    
        except Exception as e:
            print(f"Hubo un error en la busqueda de la sala. {e}")
            
        return desct_office

# funcion para cierre limpio de ejecucion
def clean_shutdown(node, executor):

    try:
        executor.remove_node(node)
        node.destroy_node()
    except Exception as e:
        print(f"Error while removing or destroying node: {e}")

    try:
        executor.shutdown()
    except Exception as e:
        print(f"Error while shutting down executor: {e}")

    try:
        rclpy.shutdown()
    except Exception as e:
        pass

# Funcion main de ejecucion de nodo
def main():
    # Inicializacion y asignacion de nodo
    rclpy.init()
    node = multiagentOllamaNode()
    
    # Configuracion y activacion automatica de nodo
    node.trigger_configure()
    node.trigger_activate()

    # Ejecutor de nodo
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")
    finally:
        clean_shutdown(node, executor)


if __name__ == '__main__':
    main()