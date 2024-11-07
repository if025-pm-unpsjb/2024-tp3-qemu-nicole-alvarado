#! /bin/sh
docker run --rm -v $(pwd):/app -w /app --network host rtsg make qemu-gdb
