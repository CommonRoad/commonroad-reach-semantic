include(FetchContent)


FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git
        GIT_TAG "feature/obstacle-bounds-via-ccs"
#        URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)
