include(FetchContent)

FetchContent_Declare(
    crreach
    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-reachable-set.git
        GIT_TAG c69591ebc6da957d965eac0ad08340be88dbc935
)

FetchContent_MakeAvailable(crreach)

# Only build targets of crreach if they are required by one of our targets
set_property(DIRECTORY ${crreach_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
