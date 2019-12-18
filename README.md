
# `proj-parking`

## Building

From the root directory of the project, run:

`docker -H BOTNAME.local build -t duckietown/proj-parking:BRANCH_NAME .`

## Running

From the root directory of the project, run:

`docker -H BOTNAME.local run -it --rm -v /data:/data --privileged --network=host duckietown/proj-parking:BRANCH_NAME`
