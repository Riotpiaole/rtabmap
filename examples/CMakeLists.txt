
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
   ADD_SUBDIRECTORY( RGBDMapping )
   ADD_SUBDIRECTORY( WifiMapping )
   ADD_SUBDIRECTORY( NoEventsExample )
   ADD_SUBDIRECTORY( Export )
   ADD_SUBDIRECTORY( scheduling  )
   ADD_SUBDIRECTORY( eventExample )
   ADD_SUBDIRECTORY( RealSenseDemo )
   ADD_SUBDIRECTORY( occupancy_grid_export )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping and WifiMapping examples will not be built...")
ENDIF()


