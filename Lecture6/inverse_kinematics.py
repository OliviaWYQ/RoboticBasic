"""Compatibility entry point used by test_inverse_kinematics.py.

The lab submission contains separate analytic and numerical implementations.
The supplied test node imports this original module name, so it uses the
analytic implementation by default.
"""

from inverse_kinematics_analytic import inverse_kinematics


__all__ = ["inverse_kinematics"]
