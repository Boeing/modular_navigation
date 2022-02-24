# Use these base error classes for each of the policies
# Decisions can be made based on the expected error outcomes


class BaseError:
    def __init__(self, msg):
        self.msg = msg


class ControlPolicyError(BaseError):
    pass


class CostmapPolicyError(BaseError):
    pass


class WaypointPolicyError(BaseError):
    pass
