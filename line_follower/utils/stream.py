import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))   # UDP: nothing is sent, it only picks the outgoing interface
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'   # no network route; the stream still serves on every interface
    finally:
        s.close()


class MJPEGStream:
    def __init__(self, port=8080, path='/camera', fps=15):
        self.port = port
        self.path = path
        self._period = 1.0 / fps
        self._frame = None
        self._lock = threading.Lock()

        stream = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != stream.path:
                    self.send_response(404)
                    self.end_headers()
                    return
                stream._serve(self)

            def log_message(self, format, *args):
                pass

        self._server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        self._server.daemon_threads = True   # open browser tabs must not keep the node alive
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def url(self):
        return f'http://{get_local_ip()}:{self.port}{self.path}'

    def update(self, frame):
        with self._lock:
            self._frame = frame

    def close(self):
        self._server.shutdown()
        self._server.server_close()

    def _serve(self, handler):
        handler.send_response(200)
        handler.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        handler.send_header('Cache-Control', 'no-cache, private')
        handler.send_header('Pragma', 'no-cache')
        handler.end_headers()

        try:
            while True:
                with self._lock:
                    frame = self._frame

                if frame is not None:
                    ok, jpeg = cv2.imencode('.jpg', frame)
                    if ok:
                        handler.wfile.write(b'--frame\r\n')
                        handler.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        handler.wfile.write(jpeg.tobytes())
                        handler.wfile.write(b'\r\n')

                time.sleep(self._period)
        except (BrokenPipeError, ConnectionResetError):
            pass
