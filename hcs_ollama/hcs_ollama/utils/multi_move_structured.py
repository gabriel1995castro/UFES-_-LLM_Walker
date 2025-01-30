# Libraries for configuring output structures of agents
from typing_extensions import TypedDict
from pydantic import BaseModel
from pydantic import Field
from typing import Any

class EvaluatorResponse(BaseModel):
    is_valid_request: bool = Field(description="True if it corresponds to a task within your capabilities. False otherwise.")
    count_objectives: int = Field(description="Identify the number of objectives required to fulfill the request.")
    direct_requests: Any = Field(description="Translate the request into a direct phrase with a specific objective and action, do this for each objective of the request and put as list.")


class InfoResponse(BaseModel):
    objective_name:str = Field(description="Return the only subject name, object name or place name in the request.")
    name_place: str = Field(description="Provide the specific place name (NOT USE PERSON NAMES OR OBJECTS NAMES) related to the requested information. If have_information is False, return an empty string ('').")

class LocalizationState(TypedDict):
    user_request: str
    validate_request: bool
    direct_requests: list
    objective_name: str
    place_search: str
    objective_coordinates: dict