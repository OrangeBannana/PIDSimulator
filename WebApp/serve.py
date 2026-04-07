#!/usr/bin/env python3
import http.server
import socketserver
import os
import webbrowser
from pathlib import Path

PORT = 8000
DIRECTORY = Path(__file__).parent

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIRECTORY), **kwargs)

os.chdir(DIRECTORY)

with socketserver.TCPServer(("", PORT), MyHTTPRequestHandler) as httpd:
    url = f"http://localhost:{PORT}"
    print(f"Server running at {url}")
    print("Press Ctrl+C to stop")
    webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped")
