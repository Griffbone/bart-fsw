FROM debian:bullseye

RUN apt update && \
    apt clean && \
    apt install -y \
      build-essential \
      wget=1.21-1+deb11u1 \
      curl=7.74.0-1.3+deb11u13 \
      make=4.3-4.1 \
      gdb-multiarch=10.1-1.7 \
      openocd=0.11.0~rc2-1 \
      stlink-tools=1.6.1+ds-3 \
      git=1:2.30.2-1+deb11u3 \
      clang-format=1:11.0-51+nmu5 \
      neovim=0.4.4-1

WORKDIR /home

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

RUN tar xJf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz -C /usr/share

RUN ln -s /usr/share/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc

RUN ln -s /usr/share/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++

RUN ln -s /usr/share/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb

RUN ln -s /usr/share/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size

RUN ln -s /usr/share/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy

RUN rm -rf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

WORKDIR /home/template

RUN echo 'export PS1="\[$(tput bold)\]\[$(tput setaf 1)\][\[$(tput setaf 3)\]\u\[$(tput setaf 2)\]@\[$(tput setaf 4)\]\h \[$(tput setaf 5)\]\W\[$(tput setaf 1)\]]\[$(tput setaf 7)\]\\$ \[$(tput sgr0)\]"' >> /etc/bash.bashrc

CMD bash
