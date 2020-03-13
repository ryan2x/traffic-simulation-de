#!/usr/bin/env bash

docker run --rm -i --name stem-2020 -p 9908:80 \
  -v $PWD:/usr/share/nginx/html:ro \
  nginx

