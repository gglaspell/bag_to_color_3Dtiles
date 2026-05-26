FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3 \
    python3-venv \
    python3-pip \
    libegl1 \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

RUN pip install --no-cache-dir \
    rosbags \
    open3d \
    numpy \
    scipy \
    tqdm \
    Pillow \
    pyproj \
    py3dtiles

COPY bag_to_tileset.py .

ENTRYPOINT ["/opt/venv/bin/python", "/app/bag_to_tileset.py"]
