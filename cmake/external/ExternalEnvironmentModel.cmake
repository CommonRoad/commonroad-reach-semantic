include(FetchContent)

# Force the environment model to always use the external drivability checker
set(EXTERNAL_CRDC_FORCE ON)

FetchContent_Declare(
        EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:commonroad-traffic-rules/environment-model.git
        # TODO: Update this once the branch feature/reach-semantic-compatibility in the environment model is merged
        GIT_TAG 52f6e03ee773f72f0233ab5d452020e3bf7b60bf
        #    URL /home/lercher/tum/commonroad/environment-model
)

FetchContent_MakeAvailable(EnvironmentModel)

# Only build targets of environment model if they are required by one of our targets
set_property(DIRECTORY ${EnvironmentModel_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
