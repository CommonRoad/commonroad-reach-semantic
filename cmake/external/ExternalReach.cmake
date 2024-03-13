include(FetchContent)

FetchContent_Declare(
    crreach
    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-reachable-set.git
        GIT_TAG sync-dependencies-with-env-model
        #    URL /home/lercher/tum/commonroad-reachable-set
)

FetchContent_MakeAvailable(crreach)

# Only build targets of crreach if they are required by one of our targets
set_property(DIRECTORY ${crreach_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
