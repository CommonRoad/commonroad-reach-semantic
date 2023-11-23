## Updating the Docker Image for CI

If we need any new dependencies, we have to update the Docker image that is used for CI.

To update the Docker image, update the tag in [build_docker.sh](./build_docker.sh) to a new version.
Then simply run
```bash
./build_docker.sh
```
and enter your LRZ GitLab credentials when prompted.
