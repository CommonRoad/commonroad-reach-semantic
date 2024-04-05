include(FetchContent)

# Force the environment model to always use the external drivability checker
set(EXTERNAL_CRDC_FORCE ON)

FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git
        GIT_TAG 3030c8501ed9d7c48925ecb642d496a60bd431ec
        #    URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
