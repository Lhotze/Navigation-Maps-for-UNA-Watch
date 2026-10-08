# Builds map packages without installing anything on the host (Windows, macOS, Linux):
#   docker build -t offlinenav-maps .
#   docker run --rm -v "$PWD/maps:/maps" offlinenav-maps europe/germany/bremen -o /maps
FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends g++ && rm -rf /var/lib/apt/lists/*
WORKDIR /tools
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY *.py *.cpp ./
ENTRYPOINT ["python", "make_map.py"]
