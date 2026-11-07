FROM ubuntu:22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install matplotlib

WORKDIR /app
COPY . .

RUN mkdir -p build && cd build && cmake .. && make

CMD ["./build/MotorControlSIL"]
