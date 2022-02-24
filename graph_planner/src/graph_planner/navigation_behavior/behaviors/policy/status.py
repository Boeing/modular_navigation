from enum import Enum


class Status(Enum):
    SUCCESS = 'success'
    RUNNING = 'running'
    ERROR = 'error'
    INTERVENTION = 'intervention'
    NOT_RUNNING = 'not_running'  # Conditions not met so not running
    UNKNOWN = 'unknown'
