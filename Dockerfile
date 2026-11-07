FROM ubuntu:22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    cmake \
    make \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install matplotlib

WORKDIR /app
COPY . .

RUN mkdir -p build && cd build && cmake .. -DCMAKE_C_COMPILER=gcc && make

CMD ["./build/MotorControlSIL"]
