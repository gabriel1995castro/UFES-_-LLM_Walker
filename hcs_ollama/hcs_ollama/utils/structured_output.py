# Seccion de importe de librerias
# Librerias par manejo de tipo de datos
from pydantic import BaseModel
from pydantic import Field

# Seccion de creacion de clases 
# Respuesta estructurada de agente evaluadador
class EvaluatorResponseStructure(BaseModel):
    is_movement_request: str = Field(description="Indicates if the task involves movement, position adjustment, or guidance. Answer with 'Yes' or 'No'.")
    translate_task: str = Field(description="Translate the task into a concise phrase that directly identifies the request. If not a movement request, this field will be 'None'.")
    location_objective: str = Field(description="Extract the final location objective from the user request. If is_movement_request is 'No', this field will be 'None'.")

class LocalizatorResponseStructure(BaseModel):
    known_info: bool = Field(description="Indicates if specific information is available for the user's request. Set to False if the information is unavailable.")
    location_name: str = Field(description="Name of the location related to the user's request. Use name of section from context.")

class ExecutorResponseStructure(BaseModel):
    coordinates: dict = Field(description="Indicates the goal objective for request.")
    actual_state: str = Field(description="State of general evaluation.")
