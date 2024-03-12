include(FetchContent)

FetchContent_Declare(
    crreach
    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-reachable-set.git
        GIT_TAG 9da18616b5c803e825248bc011847bbd22bde54d
)

FetchContent_MakeAvailable(crreach)

# Only build targets of crreach if they are required by one of our targets
set_property(DIRECTORY ${crreach_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
