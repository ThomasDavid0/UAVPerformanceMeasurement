from loguru import logger
import sys

def configure_logger(**args):
    logger.remove(0)
    

logger.add(
    sys.stdout, colorize=True, level='INFO',
    format="<green>{name}, {module}, {thread}, {time:HH:mm:ss}</green> <level>{message}</level>"
)

logger.bind(test='safsdc').info('{test} sdcvcsd')