#! /bin/sh
docker run -t -i --rm -v $(pwd):/app -w /app --network host rtsg make $@
