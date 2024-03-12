include(FetchContent)


FetchContent_Declare(
        EnvironmentModel
        #        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git
        #        GIT_TAG "f1a4d43998fc47616f261525cdd659149cdeb2f5"
        URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)
