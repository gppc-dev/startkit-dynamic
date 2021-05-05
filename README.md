
# TLDR

* Participants push commits to their repository and we will be able to compile, run and evaluate.

# Details on the server side

## I/O Setup

* All `stdout` from participant's program are redirected to a file `run.stdout`

* All `stderr` from participant's are redirected to a file `run.stderr`

* The results of benchmark (i.e. `../${exec} -run <map> <scen>`) are written to `result.csv`

* All these files are in docker, and will backup to server so that we can hide/reveal information to participants.

## Execution Setup

* The server run all scripts under certain constraints, e.g. time limit, memory limit.

* A execution will be killed by server when it exceeds limits, and participants will get `Time Limit Exceed` / `Memory Limit Exceed` verdict.

* If a execution crashed, participants will get `Run Time Error` verdict with few or none hints.

* Participants can submit their solution and execute in `dev` mode for testing purpose, in `dev` mode:
  * we only test a small set of simple map
  * we will reveal all `stdout` and `stderr` to participants
  * this mode is to eliminate compile errors and illegal output format.
  * although participants can do this locally, we encourage everyone to submit on `dev` mode at least once.


## Evaluation Workflow

### TLDR
1. Build docker image based on Dockerfile in submission repo.
2. Start the container in background.
3. Run pre-processing for debug maps.
4. Run validation for debug scenarios.
5. Run pre-processing for benchmark maps.
6. Run validation for benchmark scenarios.
7. Run benchmark for benchmark scenarios.
8. Submit final result.


### Details


* Setting up environment: 
  * The docker image working directory should be set to the directory where executables are.
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
  * if the execution not failed, we will send output to our `validator`, otherwise see in [Execution Setup](##execution-setup)
  * the `validator` read from `stdout` file and show verdict message to participants
  * `validator` produces 3 types of verdicts: 
    * `correct`: all paths are valid and lengths are optimal
    * `suboptimal`: all paths are valid, but lengths may not optimal
    * `wrong answer`: there are invalid paths

* Running executable for benchmarking: `<prefix> ./run -run $${map_path} ${scenario_path}`
  * we will track time/memory usage and publish results
