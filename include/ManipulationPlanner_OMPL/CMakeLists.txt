set(hdrs ManipulationPlanner_OMPL.h MotionPlanner.h CollisionChecker.h p3-uav-helper.h p4-arm-helper.h
    PARENT_SCOPE
    TrajectoryPlannerSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

