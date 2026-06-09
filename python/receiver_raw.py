#!/usr/bin/env python3
"""
RTL-SDR Raw Data Receiver
Listen on 127.0.0.1:23568, print all received raw data
"""

import json
import logging
from http.server import HTTPServer, BaseHTTPRequestHandler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("receiver_raw")


class DataHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        if self.path != "/api/service":
            self.send_error(404, "Not Found")
            return

        content_length = int(self.headers.get("Content-Length", 0))
        if content_length == 0:
            self.send_error(400, "Empty Body")
            return

        body = self.rfile.read(content_length)
        try:
            payload = json.loads(body)
            event = payload.get("event", "unknown")
            data = payload.get("data", {})

            logger.info("=== Event: %s ===", event)
            logger.info("Data: %s", json.dumps(data, indent=2, ensure_ascii=False))

        except json.JSONDecodeError as e:
            logger.error("Invalid JSON: %s", e)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')

    def log_message(self, format, *args):
        pass


def main():
    host = "127.0.0.1"
    port = 23568

    server = HTTPServer((host, port), DataHandler)
    server.timeout = 0.5

    logger.info("Raw data receiver started: http://%s:%d/api/service", host, port)
    logger.info("Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        server.shutdown()
        server.server_close()
        logger.info("Service stopped")


if __name__ == "__main__":
    main()
