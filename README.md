
# TLDR

* Participants push commits to the repository and we will be able to run.

# Details on the server side

* Setting up environment: 
  * `docker build -t <image name> <dockerfile>`: build docker image based on a default/user-defined dockerfile
  * `docker run -d <image> ...`: start the container in background

* All following commands are running in docker, and have prefix: `docker container exec -it <container name>`

* Building executable:
  * `<prefix> ./compile.sh`: run user provided compile script
  * user can read all outputs for debugging

* Running preprocessing:
  * `<prefix> ./preprocessing.sh`: run user provided preprocessing script
  * To discuss: user can read none/part/full outputs?

* Running executable with validator
  * `<prefix> bash -c "./run --validate ${other flags} | ./validator"`
  * the `validator` read from `stdin` and print verdict message to `stdout`, and user can read the full verdict message from the site

* Running executable for benchmarking: `<prefix> ./run ${other flags}`
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

