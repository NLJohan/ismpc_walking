#pragma once

// Small, self-contained bridge between Python (via the Cython extension in
// this same directory) and the live Walking_controller/ISMPC_Solver C++
// objects. Deliberately NOT part of mc_rtc's own bindings.
//
// IMPORTANT: this file deliberately does NOT #include mc_control's own
// Cython-generated .pxd-derived declarations (i.e. we never `cimport
// mc_control.mc_control` from the .pyx side either). Doing so transitively
// requires mc_rbdyn_wrapper.hpp / mc_control_wrapper.hpp, which mc_rtc's own
// build treats as private and never installs -- any downstream Cython
// extension that does `cimport mc_control.mc_control` hits this wall.
//
// Instead we use Cython's "public api" mechanism directly: mc_control.pxd
// declares `MCController` as `cdef public api class ... [object
// MCControllerObject, type MCControllerType]`, which makes Cython generate
// two small, self-contained headers (mc_control_api.h, mc_control.h) whose
// only job is to let external C/C++ code access that class's raw struct
// layout and PyTypeObject, without needing the rest of mc_rtc's Cython
// source graph. Those two headers are vendored here (copied verbatim from
// mc_rtc's own build tree) precisely because they are generated artifacts
// meant for this kind of external consumption.

#include <mc_control/mc_controller.h>
#include <mc_control/mc_python_controller.h>
#include "ismpc_walking/Walking_controller.h"

#include "vendored/mc_control.h"
#include "vendored/mc_control_api.h"

/**
 * @brief Unwrap a Python mc_control.MCController object into the raw C++
 * pointer it wraps, or nullptr if py_ctl isn't actually one (wrong type,
 * or the mc_control module's type import failed).
 */
inline mc_control::MCController * ismpc_walking_unwrap_mc_controller(PyObject * py_ctl)
{
  if(import_mc_control__mc_control() != 0)
  {
    PyErr_Clear();
    return nullptr;
  }
  if(py_ctl == nullptr || !PyObject_TypeCheck(py_ctl, &MCControllerType))
  {
    return nullptr;
  }
  return reinterpret_cast<MCControllerObject *>(py_ctl)->base;
}

/**
 * @brief Push RL-set CoM-height sine parameters into the running ISMPC solver.
 *
 * @param py_ctl      A Python mc_control.MCController object (whatever
 *                    mc_mjlab's worker code holds via
 *                    MCGlobalController.controller()).
 * @param offset      CoM height the sine rides on (m). Caller-side (mc_mjlab)
 *                    responsibility: already clipped to a safe positive range.
 * @param amplitude   Sine amplitude (m). Caller-side responsibility: already
 *                    derived as amplitude_ratio * offset, so offset - amplitude
 *                    >= 0 always -- this function does not re-check that.
 * @param frequency   Sine frequency (Hz).
 * @param phase       Sine phase offset (rad).
 * @return true if py_ctl was actually a Walking_controller and the call
 *         landed, false otherwise (caller should treat false as "nothing
 *         happened", not necessarily an error -- see notes in step_env).
 */
inline bool ismpc_walking_set_com_height_sine_params(PyObject * py_ctl,
                                                       double offset,
                                                       double amplitude,
                                                       double frequency,
                                                       double phase)
{
  auto * ctl = ismpc_walking_unwrap_mc_controller(py_ctl);
  if(ctl == nullptr)
  {
    return false;
  }
  auto * walking = dynamic_cast<Walking_controller *>(ctl);
  if(walking == nullptr)
  {
    return false;
  }
  walking->ismpc_solver().SetCoMHeightSineParams(offset, amplitude, frequency, phase);
  return true;
}

/**
 * @brief Read back CoM_height[0] -- the reference the whole-body controller
 * is tracking right now. Useful for reward/observation terms later.
 */
inline bool ismpc_walking_get_com_height_ref(PyObject * py_ctl, double & value)
{
  auto * ctl = ismpc_walking_unwrap_mc_controller(py_ctl);
  if(ctl == nullptr)
  {
    return false;
  }
  auto * walking = dynamic_cast<Walking_controller *>(ctl);
  if(walking == nullptr)
  {
    return false;
  }
  const auto & profile = walking->ismpc_solver().CoM_height_vec();
  if(profile.empty())
  {
    return false;
  }
  value = profile[0];
  return true;
}

/**
 * @brief Whether the last GetWalkingParameters() QP solve succeeded.
 */
inline bool ismpc_walking_qp_succeeded(PyObject * py_ctl, bool & value)
{
  auto * ctl = ismpc_walking_unwrap_mc_controller(py_ctl);
  if(ctl == nullptr)
  {
    return false;
  }
  auto * walking = dynamic_cast<Walking_controller *>(ctl);
  if(walking == nullptr)
  {
    return false;
  }
  value = walking->ismpc_solver().QPsucceeded();
  return true;
}