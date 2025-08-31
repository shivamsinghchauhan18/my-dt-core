#!/usr/bin/env python3
"""
Local namespace shim for duckietown_msgs

This repository contains enhanced utilities and docs under the Python package
name "duckietown_msgs". However, the canonical Duckietown message and service
definitions are provided by the ROS-generated package installed with the base
image (apt). A plain Python package placed on sys.path would otherwise shadow
the ROS-generated "duckietown_msgs" package, breaking imports such as
"from duckietown_msgs.msg import BoolStamped" or "from duckietown_msgs.srv import ChangePattern".

To avoid shadowing and keep compatibility, we declare this as a namespace
package and extend its search path to include the ROS-installed locations.
This allows importing standard messages/services from duckietown_msgs.msg/srv
while still exposing our optional enhanced modules (e.g., duckietown_msgs.msg.enhanced).
"""

from pkgutil import extend_path
import os

# Merge with any other duckietown_msgs distributions (e.g., ROS-generated)
__path__ = extend_path(__path__, __name__)

# Heuristic: prefer the ROS-generated distribution (contains msg/__init__.py)
# over any overlay portions that might only provide utilities. This ensures that
# `from duckietown_msgs.msg import ...` resolves to the proper message package.
def _rank(p):
	try:
		return 0 if os.path.isfile(os.path.join(p, 'msg', '__init__.py')) else 1
	except Exception:
		return 2

try:
	__path__ = type(__path__)(sorted(list(__path__), key=_rank))
except Exception:
	# Fallback: leave __path__ as-is if sorting fails
	pass

# Optional version tag for utilities in this overlay (does not reflect ROS msgs version)
__version__ = "1.0.0-overlay"
