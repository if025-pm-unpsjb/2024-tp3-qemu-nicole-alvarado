FROM ubuntu:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    make \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /home/dev

WORKDIR /home/dev

RUN mkdir arm-none-eabi-gcc qemu openocd

RUN wget https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.3.1-1.1/xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz \
    && wget https://github.com/xpack-dev-tools/qemu-arm-xpack/releases/download/v8.2.6-1/xpack-qemu-arm-8.2.6-1-linux-x64.tar.gz \
    && wget https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.11.0-4/xpack-openocd-0.11.0-4-linux-x64.tar.gz \
    && tar xfz xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz -C arm-none-eabi-gcc --strip-components 1 \
    && tar xfz xpack-qemu-arm-8.2.6-1-linux-x64.tar.gz -C qemu --strip-components 1 \
    && tar xfz xpack-openocd-0.11.0-4-linux-x64.tar.gz -C openocd --strip-components 1 \
    && rm -rf *.tar.gz

ENV PATH "/home/dev/arm-none-eabi-gcc/bin:/home/dev/qemu/bin:/home/dev/openocd:$PATH"
