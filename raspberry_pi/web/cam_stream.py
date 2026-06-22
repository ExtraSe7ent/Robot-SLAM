#!/usr/bin/env python3
"""
MJPEG Streaming Server using rpicam-vid.
Runs independently of ROS 2, optimized for low latency and memory safety.
"""
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WIDTH, HEIGHT, FPS = 640, 480, 30
BOUNDARY = b'jpgboundary'
PORT = 8080
MAX_BUF_SIZE = 1048576 # 1MB limit to prevent memory leak

class CamStream:
    def __init__(self):
        self.frame = b''
        self.condition = threading.Condition()
        self.proc = self._start_rpicam()
        threading.Thread(target=self._read_frames, daemon=True).start()
        print(f'[CAM] rpicam-vid started at {WIDTH}x{HEIGHT}@{FPS}fps')

    def _start_rpicam(self):
        """Helper method to start or restart the rpicam-vid process."""
        return subprocess.Popen([
            'rpicam-vid', '-t', '0',
            '--width', str(WIDTH), '--height', str(HEIGHT),
            '--framerate', str(FPS),
            '--codec', 'mjpeg', '--nopreview', '-o', '-'
        ], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def _read_frames(self):
        buf = b''
        while True:
            chunk = self.proc.stdout.read(4096)
            if not chunk:
                self._restart()
                buf = b''
                continue
            
            buf += chunk
            
            # Memory leak protection: clear buffer if it grows too large
            if len(buf) > MAX_BUF_SIZE:
                print('[CAM] Warning: Buffer overflow detected. Flushing buffer.')
                buf = b''
                continue
                
            while True:
                start = buf.find(b'\xff\xd8')
                if start == -1:
                    break
                end = buf.find(b'\xff\xd9', start + 2)
                if end == -1:
                    break
                    
                # Extract the complete JPEG frame
                with self.condition:
                    self.frame = buf[start:end+2]
                    self.condition.notify_all()
                
                # Truncate buffer to process next frame
                buf = buf[end+2:]

    def _restart(self):
        print('[CAM] rpicam-vid crashed, restarting in 2s...')
        try:
            self.proc.kill()
        except Exception:
            pass
        time.sleep(2)
        self.proc = self._start_rpicam()
        print('[CAM] rpicam-vid restarted successfully')

    def get_frame(self):
        with self.condition:
            # Wait up to 1 second for a new frame
            self.condition.wait(timeout=1.0)
            return self.frame

cam = CamStream()

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.split('?')[0] == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={BOUNDARY.decode()}')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    frame = cam.get_frame()
                    if frame:
                        self.wfile.write(b'--' + BOUNDARY + b'\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
            except Exception:
                # Client disconnected
                pass
        else:
            self.send_error(404)

    def log_message(self, *args):
        # Disable default logging to keep terminal output clean
        pass

if __name__ == '__main__':
    print(f'[CAM] MJPEG server is running: http://0.0.0.0:{PORT}/stream.mjpg')
    try:
        ThreadingHTTPServer(('', PORT), StreamHandler).serve_forever()
    except KeyboardInterrupt:
        print("\n[CAM] Shutting down...")
        try:
            cam.proc.kill()
        except Exception:
            pass
