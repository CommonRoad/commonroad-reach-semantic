include(FetchContent)

FetchContent_Declare(
        crreach
        GIT_REPOSITORY https://github.com/CommonRoad/commonroad-reachable-set.git
        GIT_TAG 3b59d54f591e578418063ede614adad020158311
        # GIT_TAG develop
        # URL /home/lercher/tum/commonroad-reachable-set
)

FetchContent_MakeAvailable(crreach)

# Only build targets of crreach if they are required by one of our targets
set_property(DIRECTORY ${crreach_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
