if(lcm_FOUND)
  drake_install_headers(
    bicycle_car_parameters.h
    bicycle_car_parameters_translator.h
    bicycle_car_state.h
    bicycle_car_state_translator.h
    driving_command.h
    driving_command_translator.h
    euler_floating_joint_state.h
    euler_floating_joint_state_translator.h
    idm_planner_parameters.h
    maliput_railcar_params.h
    maliput_railcar_params_translator.h
    maliput_railcar_state.h
    maliput_railcar_state_translator.h
    simple_car_params.h
    simple_car_params_translator.h
    simple_car_state.h
    simple_car_state_translator.h
    )
endif()
