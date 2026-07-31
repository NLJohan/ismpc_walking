# distutils: language = c++
from libcpp cimport bool as cppbool

# NOTE: functions take `object`, not `mc_control.MCController`, deliberately.
# The bridge header (ismpc_walking_bridge.h) unwraps the raw PyObject* itself
# via the vendored Cython public-API headers -- this file has no `cimport
# mc_control...` at all, so it never triggers mc_rtc's uninstalled
# mc_control_wrapper.hpp / mc_rbdyn_wrapper.hpp dependency chain.

cdef extern from "ismpc_walking_bridge.h":
  cppbool ismpc_walking_set_com_height_sine_params(object py_ctl,
                                                     double offset,
                                                     double amplitude,
                                                     double frequency,
                                                     double phase) except+

  cppbool ismpc_walking_get_com_height_ref(object py_ctl,
                                            double & value) except+

  cppbool ismpc_walking_qp_succeeded(object py_ctl,
                                      cppbool & value) except+
