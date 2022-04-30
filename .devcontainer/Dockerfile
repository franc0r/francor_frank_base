ARG VARIANT="latest"
FROM franc0r/ros2-francor-gui:${VARIANT}

RUN apt-get update \
   && apt-get install -q -y clangd-12 clang-format-12 gdb \
   && ln -s /usr/bin/clang-format-12 /usr/bin/clang-format \
   && ln -s /usr/bin/clangd-12 /usr/bin/clangd \
   && rm -rf /var/lib/apt/lists/* \
   && mkdir -p /home/user/ros2_ws/src