# This file is mostly for providing standard outcome format
import copy
from typing import Optional

from .error import BaseError
from .parameters import BaseParameters
from .status import Status
from ..shared_data import SharedData


class BasePolicyOutcome:
    def __init__(self, status: Optional[Status], parameters: Optional[BaseParameters], error: Optional[BaseError]):
        self.status: Optional[Status] = status
        self.parameters: Optional[BaseParameters] = parameters
        self.error: Optional[BaseError] = error


class BasePolicy:
    def __init__(self, shared_data: SharedData, name: Optional[str] = None):
        self.shared_data = shared_data
        self.shared_data_updated = False

        self.__name = name

    @property
    def name(self):
        # type: () -> str
        if self.__name:
            return copy.copy(self.__name)
        return self.__class__.__name__

    def set_shared_data(self, shared_data: SharedData):
        self.shared_data = shared_data
        self.shared_data_updated = True
