
# TLDR

* Participants push commits to the repository and we will be able to run.

# Details on the server side

* Setting up environment: 
  * `docker build -t <image name> <dockerfile>`: build docker image based on a default/user-defined dockerfile.
  * Building executable:
    * The dockerfile include a RUN command that running a user provided compile script `./compile.sh` to build executable.
  * `docker run -d --name <container name> <image name> ...`: start the container in background

* All following commands are running in docker, and have prefix: `docker container exec <container name>`

* Running preprocessing:
  * `<prefix> ./run -pre ${map_path} none`: run user provided preprocessing script
  * To discuss: user can read none/part/full outputs?

* Running executable with validator
  * `<prefix> ./run -check ${map_path} ${scenario_path} `
  * the run will write output to a file and the server copy the output from container to host server and use `validator` to validate the output.

* Running executable for benchmarking: `<prefix> ./run -run $${map_path} ${scenario_path}`
  * we will track time/memory usage

# Evaluation Workflow
* The docker image working directory should be set to the directory where executables are.

1. Build docker image based on dockerfile in submission repo.
2. Start the container in background.
3. Run pre-processing for debug maps.
4. Run validation for debug scenarios.
5. Run pre-processing for benchmark maps.
6. Run validation for benchmark scenarios.
7. Run benchmark for benchmark scenarios.
8. Submit final result.

