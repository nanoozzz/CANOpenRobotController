################################## USER FLAGS ##################################

## Which platform (robot) is the state machine using?
## This is the corresponding folder name in src/hardware/platforms
set(PLATFORM M2)

## This app does not use FLNL network communication
set(USE_FLNL FALSE)

################################################################################

################## AUTOMATED PATH AND NAME DEFINITION ##########################

## StateMachine name is (and must be) the current folder name
get_filename_component(STATE_MACHINE_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
## And its relative path to the root folder is:
file(RELATIVE_PATH STATE_MACHINE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_LIST_DIR}/)

################################################################################