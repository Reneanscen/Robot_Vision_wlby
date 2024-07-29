import sys
import logging
import coloredlogs
from logging import Logger as Log


__all__ = ["logger", "Logger"]


class Logger(Log):
    def __init__(self, logger_name='GraspCV', level="debug"):
        super(Logger, self).__init__(logger_name)
        color_formatter = coloredlogs.ColoredFormatter(
            fmt='[%(name)s] %(asctime)s %(filename)s:%(lineno)-2d  %(message)s',
            level_styles=dict(
                debug=dict(color='white', bright=True),
                info=dict(color='green', bright=True),
                warning=dict(color='yellow', bright=True),
                error=dict(color='red', bold=True, bright=True),
                critical=dict(color='black', bold=True, background='red'),
            ),
            field_styles=dict(
                name=dict(color='blue'),
                asctime=dict(color='blue'),
                funcName=dict(color='blue'),
                lineno=dict(color='blue'),
            )
        )
        # logging.basicConfig()
        self.propagate = False
        ch = logging.StreamHandler(stream=sys.stdout)
        ch.setFormatter(fmt=color_formatter)
        self.addHandler(hdlr=ch)
        self.setLevel(level=eval(f"logging.{level.upper()}"))


logger = Logger()


def logger_test():
    logger.info("log test - info")
    logger.debug("log test - debug")
    logger.warning("log test - warning")
    logger.error("log test - error")
    logger.critical("log test - critical")


if __name__ == '__main__':
    logger_test()
