# Seccion de importe de libreras
from langchain_core.tools import tool
# Librerias para construccion de herramientas de uso
from pydantic import BaseModel
from pydantic import Field
from pydantic import confloat

# Primera prueba de traduccion de funciones usadas
class MoveToPose(BaseModel):
    """Specifies the coordinates on the plane where the robot should move."""

    x: confloat(ge=0, le=10) = Field(..., description="Defines the exact desired position along the x-axis for the robot to reach.")
    y: confloat(ge=0, le=10) = Field(..., description="Defines the exact desired position along the y-axis for the robot to reach.")
    theta: confloat(ge=-3.14, le=3.14) = Field(..., description="Specifies the orientation angle in radians that the robot should adopt.")

    class Config:
        str_strip_whitespace = True
        populate_by_name = True
        json_schema_extra = {
            "example": {
                "x": 5.0,
                "y": 7.5,
                "theta": 1.57
            }
        }

class RelativeMove(BaseModel):
    """Commands a relative movement for the robot."""
    
    linear: float = Field(..., description="Defines the desired linear displacement for the robot in meters.")
    angular: float = Field(..., description="Specifies the angular velocity for the robot in radians per second.")

    class Config:
        str_strip_whitespace = True
        populate_by_name = True
        json_schema_extra = {
            "example": {
                "linear": 0.5,
                "angular": 0.1
            }
        }

class RejectRequest(BaseModel):
    """Use this function if the request cannot be fulfilled."""

    reason: str = Field(..., description="Provides the reason why the request cannot be fulfilled.")
    
    class Config:
        str_strip_whitespace = True
        populate_by_name = True
        json_schema_extra = {
            "example": {
                "reason": "The robot is currently busy and cannot process the request." 
            }
        }