# Submission Instruction

## Login with your GitHub account

Login to the [competition website](https://gppc.search-conference.org/grid) with a GitHub account, and we will automatically create a private GitHub repo for you.
The repo will be the place that you submit codes to. You can click "My Repo" to open your GitHub repo page.

## Make your first submission

Clone your repo and start work on it, the startkit will already be present.
```
$ git clone git@github.com:your_repo_address
$ cd your_repo
```

## Implement your algorithm

Read the Problem_Definition.md to check the problem definitions.

Read the start kit README.md to know what files you should (not) modify, and where you should implement your algorithm.

When your implementation is ready to evaluate, add, commit, and push all your changes by running the following commands in your local repo:
```
$ git add *
$ git commit -m "Some message to describe this commit."
$ git push origin
```

If your implementation support multi-thread preprocessing, tick the "My Entry Support Multi-thread Preprocessing" option.
If ticked, the preprocessing server will assign 4 CPUs for preprocessing (otherwise only 1 CPU is assigned).

Then click the "Evaluate my codes" button on the competition website to evaluate your new submission.

## Track evaluation progress and history submission

Click the "My Submissions" button to see your submission history.
Click an entry in the history to see details and track the progress of a running evaluation.
