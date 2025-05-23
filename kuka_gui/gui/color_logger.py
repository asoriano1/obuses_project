# color_logger.py
import logging

class ColorFormatter(logging.Formatter):
    """Formatter de logging que aplica colores ANSI a los distintos niveles (Python 2 compatible)."""
    COLORS = {
        'DEBUG': '\033[94m',    # Azul
        'INFO': '\033[92m',     # Verde
        'WARNING': '\033[93m',  # Amarillo
        'ERROR': '\033[91m',    # Rojo
        'CRITICAL': '\033[95m', # Magenta
    }
    RESET = '\033[0m'

    def format(self, record):
        color = self.COLORS.get(record.levelname, '')
        message = super(ColorFormatter, self).format(record)
        if color:
            return "{}{}{}".format(color, message, self.RESET)
        return message
