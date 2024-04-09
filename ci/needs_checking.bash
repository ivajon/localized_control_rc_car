#!/usr/bin/bash

# Checks if any of the changed files are in the desired folder

files=$(git diff --name-only HEAD^ HEAD);

for file in $files; do
    if [[ $file == $1/* ]]; then
        echo "RUN_$1=\"true\"" >> $GITHUB_OUTPUT
        echo "true";
        exit;
    fi;
done;
echo "RUN_$1=\"false\"" >> $GITHUB_OUTPUT

# https://docs.github.com/en/actions/using-jobs/defining-outputs-for-jobs
