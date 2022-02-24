class NodeError(Exception):
    """
    Default node error
    """
    pass


class UnkownNodeError(NodeError):
    pass


class DuplicateNodeError(NodeError):
    """
    Duplicate node exists
    """
    pass


class DuplicateChildNodeError(DuplicateNodeError):
    """
    Duplicate children
    """
    pass


class ParentExistsNodeError(NodeError):
    """
    Parent already exists, cannot assign a new parent
    """
    pass


class EdgeError(Exception):
    """
    Default edge error
    """
    pass


class SelfEdgeError(Exception):
    """
    Start and end nodes are the same
    """
    pass


class ParallelEdgeError(Exception):
    """
    Multiple edges to and from the same nodes
    """
    pass


class AreaError(Exception):
    """
    Default area error
    """
    pass


class UnknownAreaError(AreaError):
    """
    Unknown area
    """
    pass


class DuplicateAreaError(AreaError):
    """
    Duplicate area exists
    """
    pass


class ParentInvalidAreaError(AreaError):
    """
    Parent is invalid, cannot assign parent
    """
    pass


class ParentExistsAreaError(AreaError):
    """
    Parent already exists, cannot assign a new parent
    """
    pass


class ChildInvalidAreaError(AreaError):
    """
    Child is invalid, cannot assign child
    """
    pass


class DuplicateChildAreaError(DuplicateAreaError):
    """
    Duplicate children
    """
    pass


class NotATreeAreaError(AreaError):
    """
    Area tree is no longer a tree
    """
    pass


class InconsistentTreeAreaError(AreaError):
    """
    Area tree hiearchy is inconsistent
    """
    pass
