ADD_LIBRARY( icubcontrol
    icub_controller.cpp 
    )

SET_TARGET_PROPERTIES( icubcontrol 
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib/ )

TARGET_LINK_LIBRARIES( icubcontrol ${LIBS})



ADD_EXECUTABLE( icub_controller
    icub_controller.cpp 
    main.cpp 
    )

SET_TARGET_PROPERTIES( icub_controller 
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/ )

TARGET_LINK_LIBRARIES( icub_controller ${LIBS})



ADD_EXECUTABLE( record
    icub_controller.cpp 
    record.cpp 
    )

SET_TARGET_PROPERTIES( record 
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/ )
   
TARGET_LINK_LIBRARIES( record ${LIBS})
    

 
