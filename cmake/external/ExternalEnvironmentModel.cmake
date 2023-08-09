include(FetchContent)


FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git

        GIT_TAG "develop"

#        FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(EnvironmentModel)
