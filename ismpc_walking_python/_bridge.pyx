# distutils: language = c++
# cython: language_level=3
#
# Thin Python-facing layer over ismpc_walking_bridge.h. Every function takes
# a plain Python object (expected to be an mc_control.MCController instance,
# e.g. what MCGlobalController.controller() returns in mc_mjlab's worker
# code) and forwards it, unconverted, to the C++ bridge -- which unwraps and
# type-checks it itself via Cython's public-API mechanism, without this
# module ever needing to `cimport mc_control`.

cimport ismpc_walking_python._c_bridge as c_bridge
from libcpp cimport bool as cppbool


def set_com_height_sine_params(ctl, double offset, double amplitude,
                                double frequency, double phase):
  """Push RL-set CoM-height sine params into the running ISMPC solver.

  `ctl` should be an mc_control.MCController instance. Returns True if it
  was actually a Walking_controller and the call landed, False otherwise
  (e.g. wrong controller loaded -- caller should treat that as "nothing
  happened", not as an error, since mc_mjlab may run this against other
  controllers in other tasks).
  """
  cdef cppbool ok = c_bridge.ismpc_walking_set_com_height_sine_params(
    ctl, offset, amplitude, frequency, phase
  )
  return ok


def get_com_height_ref(ctl):
  """Latest CoM_height[0] the solver computed, or None if unavailable."""
  cdef double value = 0.0
  cdef cppbool ok = c_bridge.ismpc_walking_get_com_height_ref(ctl, value)
  if not ok:
    return None
  return value


def qp_succeeded(ctl):
  """Whether the last ISMPC QP solve succeeded, or None if unavailable."""
  cdef cppbool value = False
  cdef cppbool ok = c_bridge.ismpc_walking_qp_succeeded(ctl, value)
  if not ok:
    return None
  return bool(value)


def set_reference_velocity(ctl, double vx, double vy, double wz):
  """Push the RL-commanded reference velocity (vx, vy, wz; body-frame
  linear x/y, angular z) into the running ISMPC walking controller.

  `ctl` should be an mc_control.MCController instance. Returns True if it
  was actually a Walking_controller and the call landed, False otherwise
  (e.g. wrong controller loaded -- caller should treat that as "nothing
  happened", not as an error).

  Safe to call mid-swing: Walking_controller::UpdatePlanner_input() reads
  reference_velocity fresh every call, there is no caching or sequencing
  requirement around footstep boundaries.
  """
  cdef cppbool ok = c_bridge.ismpc_walking_set_reference_velocity(
    ctl, vx, vy, wz
  )
  return ok