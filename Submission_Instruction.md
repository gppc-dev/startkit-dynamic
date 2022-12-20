# Submission Instruction

## Make your first submission

Clone the starter kit, which includes a basic sample implementation. 
```
$ git clone https://github.com/gppc-dev/startkit.git
$ cd startkit
```
Add your competition private repo to the startkit local repo.
```
$ git remote add contest_server git@github.com:your_repo_address
$ git push contest_server
```
Finally click "Evaluate my codes" button on the competition website! The server then evaluates the implementation in the repo.

## Implement your algorithm

Read the start kit README.md to know what files you are need to modify, and where you should implement your algorithm.

When your implementation is ready to evaluate, add, commit, and push all your changes by running following commands in your local repo:
```
$ git add *
$ git commit -m "Some message to describe this commit."
$ git push contest_server
```

If your implementation support multi-thread preprocessing, tick the "My Entry Support Multi-thread Preprocessing" option. 
If ticked, the preprocessing server will assign 4 cpus for preprocessing (otherwise only 1 cpu is assigned).

Then click the "Evaluate my codes" button on the competition website to evaluate your new submission.

## Track evaluation progress and history submission

Click the "My Submissions" button to see your submission history. 
Click an entry in the history to see details and track the progress of a running evaluation. 


