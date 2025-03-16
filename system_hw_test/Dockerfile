FROM alpine:latest

ARG TARGETPLATFORM
ARG BINARY

RUN apk add --no-cache libgcc libstdc++

COPY $TARGETPLATFORM/* /
ENV BINARY=$BINARY

RUN echo '#!/bin/ash' > /entrypoint.sh
RUN echo 'cat /entrypoint.sh' >> /entrypoint.sh
RUN echo 'echo " * Starting: /$BINARY $*"' >> /entrypoint.sh
RUN echo 'exec /$BINARY $*' >> /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 7446/udp
EXPOSE 7447/tcp
EXPOSE 8000/tcp

ENV RUST_LOG=info

ENTRYPOINT ["/entrypoint.sh"]

# CMD ["/bin/sh"]
# ARG TARGETPLATFORM # =linux/amd64
# ARG BINARY         #=zenoh-bridge-ros2dds
# RUN |2 TARGETPLATFORM=linux/amd64 BINARY=zenoh-bridge-ros2dds /bin/sh -c apk add --no-cache libgcc libstdc++
# RUN TARGETPLATFORM=linux/arm64 BINARY=zenoh-bridge-ros2dds apk add --no-cache libgcc libstdc++
# COPY linux/amd64/* /
# ENV BINARY=zenoh-bridge-ros2dds

