include(FetchContent)

FetchContent_Declare(
    crreach
    GIT_REPOSITORY  git@gitlab.lrz.de:cps/commonroad-reachable-set.git
    GIT_TAG         c69591ebc6da957d965eac0ad08340be88dbc935
    #GIT_TAG        development
)

FetchContent_MakeAvailable(crreach)
