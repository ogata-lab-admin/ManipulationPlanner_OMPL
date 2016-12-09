set(hdrs ManipulationPlanner_OMPL.h MotionPlanner.h
    PARENT_SCOPE
    ManipulationSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

