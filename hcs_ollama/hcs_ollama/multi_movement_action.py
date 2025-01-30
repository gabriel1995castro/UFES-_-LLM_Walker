#!/usr/bin/env python3
## This package is part of an initiative to implement the paradigm 
## of multiple agents or LLM components, designed to handle requests
## and align them with specific objectives. This node try to use the better
## practices in ros programmation.
# Author: Elio David Triana Rodriguez
# Universidade Federal do Espirito Santo
# Human Centered Systems Laboratory

# Library section for node configurate
# Ros configuration libraries
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
# Action Configuration libraries
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
# Langchain ollama connection first libraries
from langchain_ollama import ChatOllama
from langchain.callbacks.manager import CallbackManager
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain_community.document_loaders import PDFPlumberLoader
# Langchain message configuration libraries
from langchain_core.prompts import SystemMessagePromptTemplate
from langchain_core.prompts import HumanMessagePromptTemplate
from langchain_core.prompts import ChatPromptTemplate
# Langgraph node configuration libraries
from langgraph.graph import START
from langgraph.graph import END
from langgraph.graph import StateGraph
from langgraph.types import Send
# Libraries for RAG implementation
from hcs_ollama.lcad_retrieval.lcad_retriever import SentenceRetriever
# Interface message libreries
from hcs_ollama_msgs.msg import ObjectiveCoordinates
from hcs_ollama_msgs.msg import MultiAgentResponse
# Interface for action
from hcs_ollama_msgs.action import GetMultiAgentMoveRespose
# Final data type and utils libraries
from pydantic import BaseModel
from pydantic import Field
# Libraries for managing node state interfaces
from typing_extensions import TypedDict
from typing import Annotated
from typing import Any
from typing import Literal
# Utils and owm libraries
from yaml import safe_load as yaml_load
from hcs_ollama.utils.json_model import get_json_info
from hcs_ollama.utils.ollama_functions import is_model_available_locally
from hcs_ollama.utils.multi_move_structured import EvaluatorResponse
from hcs_ollama.utils.multi_move_structured import InfoResponse
from hcs_ollama.utils.multi_move_structured import LocalizationState
from hcs_ollama.utils.multiuse_functions import check_list
from time import time as t_count

# Class creation of node
class multiMoveActionNode(LifecycleNode):
    # Node structure designed to select a specific exit point for the movement requested by the robot.
    def __init__(self) -> None:
        super().__init__("multi_anget_node_movement")
        # connection ollama variable
        self.model_connection = None

        # Declaration of parameters of node with deafult values
        self.declare_parameter("ai_model", "llama3.1")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("system_config_path", "./src/hcs_ollama/ollama_config/agent_configuration.yaml")
        self.declare_parameter("dir_path", "./src/hcs_ollama/map_config/")
        self.declare_parameter("map_description", "./src/hcs_ollama/map_config/map_description.json")
        self.declare_parameter("map_semantic_description", "./src/hcs_ollama/map_config/CTXIIIDesc.pdf")

        # Inicial node validation
        self.get_logger().info(f"Initial config node. Node-Name: {self.get_name()}")

    # first configurate state of lifecycle node
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Include file reading and configuration. including first instance variables.
        self.get_logger().info(f"On Configure Node: {self.get_name()}")

        # Variables instance inicialization
        self.model_ai: str = self.get_parameter("ai_model").get_parameter_value().string_value
        self.temperature_value:float = float(self.get_parameter("temperature").get_parameter_value().double_value)

        # Systems agents configurations file read
        try:
            path_temp = self.get_parameter("system_config_path").get_parameter_value().string_value
            with open(path_temp) as stream:
                self.system_config = yaml_load(stream)
        except Exception as e:
            # None value initialization
            self.system_config = None
            self.get_logger().error(f"File error reading. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # RAG Configuration
        try:
            path_rag_files = self.get_parameter("dir_path").get_parameter_value().string_value
            # Charge files retreiver and save organizade data
            self.pdf_retriever = SentenceRetriever(use_ollama=True, encoder_model_id="nomic-embed-text")
            self.pdf_retriever.init_pdf_retriever(
                input_dir = path_rag_files, 
                data_dir="./src/hcs_ollama/retriever_data/pdf/", 
                chunk_size=500,
                overlap_chunks=50,
                force_rebuild_index=True
            )
        except:
            self.pdf_retriever = None
            self.get_logger().error(f"PDF RAG Error. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Map description charge
        try:
            path_map = self.get_parameter("map_description").get_parameter_value().string_value
            # File map read and initialize variables
            self.map_description = get_json_info(path_map)
            self.specific_spaces = self.map_description["area_layout"]["offices"]
        except Exception as e:
            self.map_description = None
            self.specific_spaces = None
            self.get_logger().error(f"File map error read. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Goal handle inicialization
        self.goal_handle = None

        # Model existing validation
        if not is_model_available_locally(self.model_ai):
            self.get_logger().error(f"Error, Model called '{self.model_ai}' no existing.")
            return TransitionCallbackReturn.FAILURE
        
        # Tansition state validate configuration
        self.get_logger().info(f"Configurate Node Successful: {self.get_name()}")
        # Seccion para creacion de publishers en caso de ser necesario
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
            This function handles the activation of the node, 
            including the setup of a multi-agent cycle for detecting 
            specific user commands. It leverages LLMs for interpreting 
            and processing the detected instructions.
        """
        self.get_logger().info(f"On activate {self.get_name()}")

        # Full connection with local model
        # Callback model config
        callback_manager = CallbackManager([StreamingStdOutCallbackHandler])
        
        # model ollama configuration
        try:
            self.model_connection = ChatOllama(
                model = self.model_ai,
                temperature = 0,
                callback_manager = callback_manager,
                num_predict = 512,
                seed = None
            )
        except Exception as e:
            self.get_logger().error(f"Error. Can't configurate the Node. {e}")
            return TransitionCallbackReturn.FAILURE
        
        # Evaluator Agent configuration
        self.evaluator_agent = self.config_agent(
            self.model_connection,
            self.system_config['agent_available_system_descriptions']['movement_evaluator'],
            self.system_config['user_available_request_template']['user_evaluator'],
            EvaluatorResponse
        )

        # Informative Agent Configuration
        self.informative_agent = self.config_agent(
            self.model_connection,
            self.system_config['agent_available_system_descriptions']['movement_informative'],
            self.system_config['user_available_request_template']['user_informative'],
            InfoResponse
        )

        # Cycle multiagent configuration
        self.multi_model = self.create_cycle_app(self.evaluator_agent, self.informative_agent)

        # Validation and action create
        try:
            self.get_logger().info("Try action create")
            self.agent_movement_action = ActionServer(
                self,
                GetMultiAgentMoveRespose,
                'get_multiagent_move_response',
                execute_callback = self.llm_execute_cb,
                goal_callback = self.goal_llm_callback,
                handle_accepted_callback = self.accepted_llm_callback,
                cancel_callback = self.cancel_llm_callback
            )
        except Exception as e:
            self.get_logger().error(f"Error in action creation. {e}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"Node Activate Successful {self.get_name()}")
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"On Deactivate {self.get_name()}")

        # Deleting active instance variables
        del self.evaluator_agent
        del self.informative_agent
        del self.multi_model

        if self.multiagent_action:
            self.multiagent_action.destroy()

        del self.model_connection

        self.get_logger().info(f"Node Deactivate Successful {self.get_name()}")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")
        
        self.model_ai:str = ""
        self.temperature_value:float = -1.0
        self.system_config = None
        self.pdf_retreiver = None
        self.map_description = None
        self.specific_spaces = None

        self.get_logger().info("Clean Node Successful")
        return TransitionCallbackReturn.SUCCESS
    
    # Action create configuration section
    # Callback for successful goal request action
    def goal_llm_callback(self, goal_request):
        self.get_logger().info('Request received successfully.\n Sending for processing:\n {0}'.format(goal_request.prompt))
        return GoalResponse.ACCEPT
    
    # Callback for successful accepted request action
    def accepted_llm_callback(self, goal_handle):
        self.get_logger().info('Goal accepted and starting execution.')
        # Inicializar hilo de ejecucion
        return goal_handle.execute()
    
    # Callback for successful cancel request action
    def cancel_llm_callback(self, goal_handle):
        self.get_logger().info('Request to cancel received successfully.')
        return CancelResponse.ACCEPT
    
    # Callback principal function from action
    def llm_execute_cb(self, goal_handle):
        # Variables called from action request
        user_request : str = str(goal_handle.request.prompt)

        # Excecution action validation
        try:
            self.get_logger().info("Multiple agents move request.")

            start_time = t_count()
            # Feedback message variable instance
            feedback_msg = GetMultiAgentMoveRespose.Feedback()

            # Status from actual response
            feedback_msg.status = "processing request"

            self.get_logger().info("First invokation of multiagents")
            # Inicializacion de primer estado de ejecucion
            initial_state_ = {"user_request": user_request}

            for event in self.multi_model.stream(initial_state_, stream_mode = "updates"):
                # Validacion de estado de procesamiento
                try:
                    # Agent name asignation
                    feedback_msg.agent_name = next(iter(event)).capitalize()
                    # Feedback response
                    feedback_msg.feedback_response = str(event[list(event.keys())[0]])
                    # iter final response
                    final_response = event[list(event.keys())[0]]
                    self.get_logger().info(f"Feedback published.")
                    self.get_logger().info(f"Request Status: {feedback_msg.status}")
                    self.get_logger().info(f"Process Agents: {feedback_msg.agent_name}")
                    self.get_logger().info(f"Actual Message: {feedback_msg.feedback_response}")
                    
                    # Feedback response using
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().debug('Feedback published.')
                except Exception as e:
                    self.get_logger().error(f"Error en solicitud de ejecucion en stream. {e}")
                
        except Exception as e:
            self.get_logger().error(f"An error occurred during streaming: {e}")
            goal_handle.abort()
            result = GetMultiAgentMoveRespose.Result()
            # result.response = ''
            return result
        
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().debug('Final feedback published.')
        # actualizar estado completado de objetivo
        goal_handle.succeed()
        # Obtener resultado final
        result = GetMultiAgentMoveRespose.Result()

        # Log de tiempo de ejecucion de proceso
        end_time = t_count()
        duration = end_time - start_time
        self.get_logger().info('Goal completed in {0:.2f} seconds.'.format(duration))

        return result
    
    # Class Extra functions
    def config_agent(self, llm_connection, system_config:str, user_config:str, structured_output = None):

        # System message structure 
        system_prompt = SystemMessagePromptTemplate.from_template(system_config)
        user_prompt = HumanMessagePromptTemplate.from_template(user_config)

        # General message Configuration
        chat_msg = ChatPromptTemplate.from_messages([
            system_prompt,
            user_prompt
        ])

        # Structure Validation response 
        if structured_output is not None:
            modelo_structured = llm_connection.with_structured_output(structured_output)
        
            model = chat_msg | modelo_structured
            return model
        
        model = chat_msg | llm_connection
        return model
    
    # Function for cycle nodes create
    def create_cycle_app(self, evaluator_agent, informative_agent, planner_agent = None):
        # Evaluator function state for evaluator agent
        def evaluator_state(state: LocalizationState):
            # Get response from evaluator agent
            response_temp = evaluator_agent.invoke({
                'request': state['user_request']
            })

            # Request validation checking multiple request
            get_validation, out_value = check_list(response_temp.direct_requests)
            if not get_validation:
                out_value = [out_value]

            return {
                "validate_request": bool(response_temp.is_valid_request),
                "direct_requests": out_value[0]
            }

        # Conditional function node for cycle
        def evaluator_router(state: LocalizationState) -> Literal["not_informative", "informative"]:
            if state['validate_request']:
                return "informative"
            else:
                "not_informative"
        
        # Informative function state for informative agent
        def informative_state(state: LocalizationState):

            # Function rag for pdf information
            original_request_retrieval = self.pdf_retriever.get_context(state['user_request'], top_k=3)
            processed_request_retrieval = self.pdf_retriever.get_context(state['direct_requests'], top_k=3)
            context_value = "\n\nRetrieved Context Fragment: \n\n".join(list(set(original_request_retrieval + processed_request_retrieval)))

            # Get response from evaluator agent
            info_response = informative_agent.invoke({
                "context": context_value,
                "objective": state['direct_requests']
            })

            # Response param values
            return {
                "objective_name":  info_response.objective_name, 
                "place_search": info_response.name_place
            }
        
        # Planner function state for planner agent
        def planner_state(state: LocalizationState):
            
            # Search specific position in map description
            if state['place_search'] == '' or state['place_search'] == None:
                return {
                    'objective_coordinates': dict()
                }
            try:
                coordinate_final = get_center_position(state['place_search'])
            except Exception as e:
                self.get_logger().error(f"Error in planner state from multiagent. {e}")
                coordinate_final = None

            return {
                'objective_coordinates': coordinate_final
            }

        # Creation of a cycle flow for collaborative multi-agent systems.
        graph = StateGraph(LocalizationState)
        graph.add_node("evaluator", evaluator_state)
        graph.add_node("informative", informative_state)
        graph.add_node("planner", planner_state)
        graph.add_edge(START, "evaluator")
        graph.add_conditional_edges(
            "evaluator", evaluator_router,
            {"not_informative": END, "informative": "informative"}
        )
        graph.add_edge("informative", "planner")
        graph.add_edge("planner", END)
        app = graph.compile()

        return app
    
def clean_shutdown(node, executor):
    """
        This function ensures a clean termination of processes 
        executed by a specific node. It focuses on managing active 
        executors and available instances involved in the operation 
        and configuration of the corresponding node.
    """
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
    node = multiMoveActionNode()
    
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