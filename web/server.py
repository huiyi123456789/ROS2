#!/usr/bin/env python3
"""
简单的视频上传服务器
- 提供静态文件服务(HTML/CSS/JS)
- 接收视频文件上传
- 支持 CORS 跨域
"""

import http.server
import cgi
import os
import sys
from pathlib import Path

UPLOAD_DIR = Path(__file__).parent / "uploads"
WEB_DIR = Path(__file__).parent
PORT = 8081


class VideoUploadHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_POST(self):
        if self.path == "/upload":
            content_type = self.headers.get("Content-Type", "")

            if "multipart/form-data" not in content_type:
                self.send_error(400, "Expected multipart/form-data")
                return

            UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={
                    "REQUEST_METHOD": "POST",
                    "CONTENT_TYPE": content_type,
                },
            )

            for field_name in form:
                file_item = form[field_name]
                if file_item.filename:
                    filename = Path(file_item.filename).name
                    filepath = UPLOAD_DIR / filename

                    with open(filepath, "wb") as f:
                        f.write(file_item.file.read())

                    file_size = filepath.stat().st_size
                    print(f"[UPLOAD] {filename} ({file_size} bytes) -> {filepath}")
                    print(f"[UPLOAD] Total files: {len(list(UPLOAD_DIR.glob('*')))}")

                    response = {
                        "status": "ok",
                        "filename": filename,
                        "size": file_size,
                        "path": str(filepath),
                    }
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(
                        __import__("json").dumps(response).encode("utf-8")
                    )
                    return

            self.send_error(400, "No file uploaded")

    def log_message(self, format, *args):
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), format % args))


def main():
    os.chdir(str(WEB_DIR))
    server = http.server.HTTPServer(("0.0.0.0", PORT), VideoUploadHandler)
    print(f"视频上传服务器已启动")
    print(f"  页面地址: http://localhost:{PORT}/")
    print(f"  上传接口: http://localhost:{PORT}/upload")
    print(f"  上传目录: {UPLOAD_DIR}")
    print(f"  按 Ctrl+C 停止服务器")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务器已停止")


if __name__ == "__main__":
    main()
