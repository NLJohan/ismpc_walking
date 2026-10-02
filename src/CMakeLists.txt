set(controller_SRC
  Walking_controller.cpp
  move_feet.cpp
  robot_state.cpp
  FootTrajectory.cpp
  ISMPC_Solver.cpp
  GUI.cpp
  logs.cpp
  joystick.cpp
  Polygon.cpp
  policy/PolicyContract.cpp
  policy/OnnxBackend.cpp
  policy/PolicyLibrary.cpp
  policy/ActionDecoder.cpp
  policy/ObservationBuilder.cpp
  policy/ControllerAdapter.cpp
  policy/ZScoreMonitor.cpp
  policy/PolicyRunner.cpp
)

set(controller_HDR
  ../include/ismpc_walking/Walking_controller.h
  ../include/ismpc_walking/FootTrajectory.h  
  ../include/ismpc_walking/ISMPC_Solver.h
  ../include/ismpc_walking/ControllerConfiguration.h
  ../include/ismpc_walking/MPC_state.h
  ../include/ismpc_walking/Polygon.h
  ../include/ismpc_walking/policy/PolicyContract.h
  ../include/ismpc_walking/policy/PolicyBackend.h
  ../include/ismpc_walking/policy/OnnxBackend.h
  ../include/ismpc_walking/policy/PolicyLibrary.h
  ../include/ismpc_walking/policy/ActionDecoder.h
  ../include/ismpc_walking/policy/ControlSink.h
  ../include/ismpc_walking/policy/RobotState.h
  ../include/ismpc_walking/policy/ObservationBuilder.h
  ../include/ismpc_walking/policy/ControllerAdapter.h
  ../include/ismpc_walking/policy/ZScoreMonitor.h
  ../include/ismpc_walking/policy/PolicyRunner.h
)

set(CONTROLLER_NAME ismpc_walking)
add_library(${CONTROLLER_NAME} SHARED ${controller_SRC} ${controller_HDR})

target_include_directories(${CONTROLLER_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
target_link_libraries(${CONTROLLER_NAME} PUBLIC pendulum_feasibility_solver::pendulum_feasibility_solver)

if(ISMPC_WITH_POLICY)
  # PRIVATE: consumers (Cython bindings, states) never see ORT headers.
  target_link_libraries(${CONTROLLER_NAME} PRIVATE onnxruntime::onnxruntime)
  target_compile_definitions(${CONTROLLER_NAME} PRIVATE ISMPC_WITH_POLICY=1)
  # Run from the build tree (fetched dir) and from the install tree (next to the controller lib).
  set_property(TARGET ${CONTROLLER_NAME} APPEND PROPERTY BUILD_RPATH "${ISMPC_ORT_LIB_DIR}")
  set_property(TARGET ${CONTROLLER_NAME} APPEND PROPERTY INSTALL_RPATH "$ORIGIN")
  # Ship libonnxruntime.so* (symlinks preserved) next to libismpc_walking.so.
  install(DIRECTORY "${ISMPC_ORT_LIB_DIR}/" DESTINATION ${MC_RTC_LIBDIR}
          FILES_MATCHING PATTERN "libonnxruntime*.so*")
endif()

install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR})

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

install(TARGETS ${CONTROLLER_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

add_subdirectory(states)
