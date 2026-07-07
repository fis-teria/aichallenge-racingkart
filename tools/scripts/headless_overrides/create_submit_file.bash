#!/bin/bash

set -euo pipefail

mkdir -p submit

tar zcvf submit/aichallenge_submit.tar.gz \
  --exclude='aichallenge_submit/log' \
  --exclude='*/__pycache__' \
  --exclude='*.pyc' \
  -C ./aichallenge/workspace/src aichallenge_submit

sha256sum submit/aichallenge_submit.tar.gz
