#!/bin/bash

PROJECT_NAME="ms"
#remote_url=$(git config --get remote.origin.url)
#REPO_NAME=$(basename $remote_url .git)
#
## fetch the current git branch name
#git_branch=$(git rev-parse --abbrev-ref HEAD)
#
## sanitize the branch name to be used as docker tag
#TAG=$(echo "$git_branch" | sed 's/[^a-zA-Z0-9\-_]//g')

REPO_NAME=rdv_zivid
TAG=develop

src_dir=$(pwd)
workspace=$src_dir/..

cd $workspace || true
docker build -t rdv-warehouse.iptime.org:7500/$PROJECT_NAME/$REPO_NAME:$TAG -f $src_dir/Dockerfile .
