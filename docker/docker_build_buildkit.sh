# this will work with buildkit installed
# the environment variables are set as build arguments in the Dockerfile
# so you don't need a shell script to execute them
docker buildx build -t flaresight:v0.1.0 . # this dot specifies the Dockerfile to be in the current directory (from where the shell script is run)