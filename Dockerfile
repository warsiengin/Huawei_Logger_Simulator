FROM ghcr.io/home-assistant/base:latest

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN apk add --no-cache python3

WORKDIR /app
COPY server.py /app/server.py

CMD ["/usr/bin/python3", "-u", "/app/server.py"]
