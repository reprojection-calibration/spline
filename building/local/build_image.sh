#!/bin/bash

set -eou pipefail

no_cache=()
target_stage=development

for i in "$@"; do
  case $i in
    --no-cache)
      no_cache="--no-cache"
      shift
      ;;
    -ts=*|--target-stage=*)
      target_stage="${i#*=}"
      shift
      ;;
    -*)
      echo "Unknown option $i"
      exit 1;
      ;;
    *)
      ;;
  esac
done

IMAGE=spline
SCRIPT_FOLDER="$(dirname "$(realpath -s "$0")")"
TAG=${IMAGE}:${target_stage}

echo "Building image with tag '$TAG' targeting stage '$target_stage'..."
DOCKER_BUILDKIT=1 docker build \
    --file "${SCRIPT_FOLDER}"/../Dockerfile \
    "${no_cache[@]}" \
    --tag "${TAG}" \
    --target "${target_stage}"-stage \
    --progress=plain \
    "${SCRIPT_FOLDER}"/../../

BUILD_SUCCESSFUL=$?

if [ ${BUILD_SUCCESSFUL} -eq 0 ]; then
    echo "Build successful: ${TAG}"
else
    echo "Build failed"
    exit 1
fi