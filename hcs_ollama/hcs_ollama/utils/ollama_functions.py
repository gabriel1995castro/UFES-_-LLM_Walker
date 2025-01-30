# Modulo para llamado de funciones utilitarias. Funciones para complementar las necesidades de los modelos desarrollados
# Seccion de importe de libreriaas
from ollama import show as ollama_show
# Importe de librerias de conversion
from base64 import b64encode
from io import BytesIO
# Importe de librerias de procesamiento de imagen
from PIL import Image

# Funcion para validacion de existencia de modelo solicitado
def is_model_available_locally(model_name: str) -> bool:
    """Funcion que recibe como parametro el nombre de un modelo de llm local, este valida su existencia en el computador
    en caso no se encuentre retorna una valor False, en caso contrario un valor True
    
    Arguments:
        - name_mode: Nombre del momdelo solicitado
    Return: 
        - exists: Valor booleano de la existencia del modelo
    """

    try:
        ollama_show(model_name)
        return True
    except Exception as e:
        print(f"No se encontro el modelo solicitado. {e}")
        return False
    
# Funcion para conversion de imagenes
def convert_to_base64(pil_image):
    """
    Convert PIL images to Base64 encoded strings

    :param pil_image: PIL image
    :return: Re-sized Base64 string
    """

    buffered = BytesIO()
    pil_image.save(buffered, format="JPEG")  # You can change the format if needed
    img_str = b64encode(buffered.getvalue()).decode("utf-8")
    return img_str
