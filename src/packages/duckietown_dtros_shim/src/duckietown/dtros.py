#!/usr/bin/env python3
"""
Minimal shim for duckietown.dtros to unblock imports in environments where the
official package cannot be installed. This implements a very small subset used
by this repository (DTROS base class, NodeType, TopicType, DTParam, ParamType)
and accepts dt_topic_type in rospy.Publisher calls.

If the official package is present, this module is ignored (namespace package
merging) because its path will be after the official one.
"""
import enum
import rospy

class NodeType(enum.Enum):
    UNKNOWN = 0
    PERCEPTION = 1
    CONTROL = 2
    PLANNING = 3
    DRIVER = 4
    DEBUG = 5

class TopicType(enum.Enum):
    UNKNOWN = 0
    PERCEPTION = 1
    CONTROL = 2
    PLANNING = 3
    DRIVER = 4
    DEBUG = 5

class ParamType(enum.Enum):
    UNKNOWN = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    STR = 4
    LIST = 5

class DTParam:
    def __init__(self, name, default=None, param_type: ParamType = ParamType.UNKNOWN, **kwargs):
        self.name = name
        self.param_type = param_type
        self._value = rospy.get_param(name, default)

    def value(self):
        return self._value

    def get_value(self):  # compatibility alias
        return self._value

    def set_value(self, v):
        self._value = v
        rospy.set_param(self.name, v)

class DTROS:
    def __init__(self, node_name: str, node_type: NodeType = NodeType.UNKNOWN):
        self.node_name = node_name
        self.node_type = node_type
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=False)

# Patch rospy.Publisher to ignore dt_topic_type kwarg gracefully
_Publisher = rospy.Publisher
def _Publisher_shim(*args, **kwargs):
    kwargs.pop('dt_topic_type', None)
    return _Publisher(*args, **kwargs)
try:
    rospy.Publisher = _Publisher_shim  # type: ignore
except Exception:
    pass
