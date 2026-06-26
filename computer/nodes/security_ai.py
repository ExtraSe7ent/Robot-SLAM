#!/usr/bin/env python3
"""
Security AI Node
- Read MJPEG stream from Pi (port 8080)
- YOLO11n-pose: person detection + pose analysis (local, free)
- Only call Gemini when pose is flagged as abnormal
- Publish /security/status (JSON) and /security/alert (Bool)
"""
import rclpy
from rclpy.node import Node
import cv2, numpy as np, threading, time, requests, base64, json, os, queue
from std_msgs.msg import String
from ultralytics import YOLO


class SecurityAI(Node):
    def __init__(self):
        super().__init__('security_ai')
        self.declare_parameter('stream_url', 'http://robotanninh.local:8080/stream.mjpg')
        self.declare_parameter('model_path', os.path.expanduser('~/yolo11n-pose.pt'))
        self.declare_parameter('gemini_cooldown', 5.0)
        self.declare_parameter('daily_api_limit', 500)
        self.declare_parameter('alert_persistence', 2)
        self.declare_parameter('yolo_conf', 0.45)
        self.declare_parameter('yolo_imgsz', 416)
        self.declare_parameter('show_window', False)
        self.declare_parameter('recheck_interval', 10.0)

        self.stream_url      = self.get_parameter('stream_url').value
        self.model_path      = self.get_parameter('model_path').value
        self.cooldown        = self.get_parameter('gemini_cooldown').value
        self.daily_limit     = self.get_parameter('daily_api_limit').value
        self.persistence     = self.get_parameter('alert_persistence').value
        self.conf            = self.get_parameter('yolo_conf').value
        self.imgsz           = int(self.get_parameter('yolo_imgsz').value)
        self.show_window     = self.get_parameter('show_window').value
        self.recheck_interval = self.get_parameter('recheck_interval').value

        self.api_key = os.environ.get('GEMINI_API_KEY', '')
        if not self.api_key:
            self.get_logger().warn(
                'GEMINI_API_KEY is not set → only running YOLO + Pose, will not call Gemini.')

        self.status_pub = self.create_publisher(String, '/security/status', 10)

        self.get_logger().info(f'Loading model: {self.model_path}')
        self.model = YOLO(self.model_path)
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        self.model(dummy, verbose=False, conf=self.conf, imgsz=self.imgsz, device='cpu')
        self.get_logger().info('YOLO warm-up done.')

        self.frame_q       = queue.Queue(maxsize=1)
        self._lock         = threading.Lock()
        self._person       = False
        self._pose_flagged = False
        self._best_jpg_b64 = ''
        self._best_conf    = 0.0
        self._suspicious   = False
        self._streak       = 0
        self._reason       = 'Initializing...'
        self._api_calls_today = 0
        self._api_date     = ''
        self._last_gemini_t = 0.0

        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._yolo_loop,    daemon=True).start()
        threading.Thread(target=self._gemini_loop,  daemon=True).start()
        self.create_timer(0.5, self._publish)

        self.get_logger().info(
            f'Security AI ready | stream={self.stream_url} | '
            f'daily_limit={self.daily_limit} Gemini calls')

    # ====================================================================
    # CAPTURE LOOP - Read MJPEG stream from Pi
    # ====================================================================
    def _capture_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Connecting: {self.stream_url}')
                r = requests.get(self.stream_url, stream=True, timeout=5)
                buf = b''
                for chunk in r.iter_content(chunk_size=4096):
                    if not rclpy.ok():
                        return
                    buf += chunk
                    if len(buf) > 2_000_000:
                        buf = b''
                        continue
                    s = buf.find(b'\xff\xd8')
                    e = buf.find(b'\xff\xd9', s + 2)
                    if s != -1 and e != -1:
                        jpg = buf[s:e + 2]
                        buf = buf[e + 2:]
                        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            try:
                                self.frame_q.get_nowait()
                            except queue.Empty:
                                pass
                            self.frame_q.put(img)
            except Exception as ex:
                self.get_logger().warn(
                    f'Stream error: {ex} → retrying in 3s',
                    throttle_duration_sec=5.0)
                time.sleep(3)

    # ====================================================================
    # POSE ANALYSIS — Local keypoint heuristics, no API calls
    # All thresholds relative to body_height → scale with camera distance
    # ====================================================================
    def _is_pose_suspicious(self, keypoints) -> tuple:
        try:
            kp   = keypoints.xy[0].cpu().numpy()
            conf = (keypoints.conf[0].cpu().numpy()
                    if keypoints.conf is not None else None)

            def get(idx, min_conf=0.4):
                if conf is not None and conf[idx] < min_conf:
                    return None
                x, y = kp[idx]
                return (float(x), float(y)) if x > 0 and y > 0 else None

            NOSE = 0
            L_SHOULDER = 5;  R_SHOULDER = 6
            L_WRIST    = 9;  R_WRIST    = 10
            L_HIP      = 11; R_HIP      = 12
            L_KNEE     = 13; R_KNEE     = 14

            nose      = get(NOSE)
            l_shoulder = get(L_SHOULDER); r_shoulder = get(R_SHOULDER)
            l_wrist    = get(L_WRIST);    r_wrist    = get(R_WRIST)
            l_hip      = get(L_HIP);      r_hip      = get(R_HIP)
            l_knee     = get(L_KNEE);     r_knee     = get(R_KNEE)

            body_height = None
            if nose and l_hip and r_hip:
                hip_y       = (l_hip[1] + r_hip[1]) / 2
                body_height = abs(hip_y - nose[1])

            thr_shoulder = body_height * 0.10 if body_height else 20
            thr_knee     = body_height * 0.25 if body_height else 60

            if l_shoulder and l_wrist and l_wrist[1] < l_shoulder[1] - thr_shoulder:
                return True, 'left hand raised above shoulder'
            if r_shoulder and r_wrist and r_wrist[1] < r_shoulder[1] - thr_shoulder:
                return True, 'right hand raised above shoulder'
            if nose and l_hip and r_hip:
                hip_y = (l_hip[1] + r_hip[1]) / 2
                if nose[1] > hip_y:
                    return True, 'abnormal forward bend'
            if l_knee and r_knee and l_hip and r_hip:
                hip_y  = (l_hip[1] + r_hip[1]) / 2
                knee_y = min(l_knee[1], r_knee[1])
                if hip_y - knee_y > thr_knee:
                    return True, 'running or moving fast'
            if l_wrist and r_wrist and l_shoulder and r_shoulder:
                arm_span      = abs(l_wrist[0] - r_wrist[0])
                shoulder_span = abs(l_shoulder[0] - r_shoulder[0])
                if shoulder_span > 0 and arm_span > shoulder_span * 2.2:
                    return True, 'arms spread abnormally wide'
            return False, 'normal pose'
        except Exception:
            return False, 'unable to read pose'

    # ====================================================================
    # YOLO LOOP — Inference + pose analysis
    # ====================================================================
    def _yolo_loop(self):
        while rclpy.ok():
            try:
                frame = self.frame_q.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                results = self.model(
                    frame, verbose=False,
                    conf=self.conf, imgsz=self.imgsz, device='cpu')

                if self.show_window:
                    try:
                        cv2.imshow("Security AI Vision", results[0].plot())
                        cv2.waitKey(1)
                    except Exception:
                        self.show_window = False
                        self.get_logger().warn('cv2.imshow failed → disabling window.')

                has_person      = False
                best_conf       = 0.0
                pose_suspicious = False
                best_jpg_b64    = ''

                for r in results:
                    if r.keypoints is None:
                        continue
                    for idx, b in enumerate(r.boxes):
                        if self.model.names[int(b.cls)] != 'person':
                            continue
                        has_person = True
                        c = float(b.conf)
                        sus, _ = self._is_pose_suspicious(r.keypoints[idx:idx + 1])
                        if sus and c > best_conf:
                            best_conf       = c
                            pose_suspicious = True
                            ok, enc = cv2.imencode(
                                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            if ok:
                                best_jpg_b64 = base64.b64encode(enc).decode()
                        elif not pose_suspicious and c > best_conf:
                            best_conf = c
                            ok, enc = cv2.imencode(
                                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            if ok:
                                best_jpg_b64 = base64.b64encode(enc).decode()

                with self._lock:
                    self._person       = has_person
                    self._pose_flagged = pose_suspicious
                    if has_person and best_jpg_b64:
                        self._best_jpg_b64 = best_jpg_b64
                        self._best_conf    = best_conf
                    if not has_person:
                        self._best_conf    = 0.0
                        self._best_jpg_b64 = ''
            except Exception as ex:
                self.get_logger().warn(f'YOLO error: {ex}', throttle_duration_sec=5.0)

    # ====================================================================
    # GEMINI LOOP — Cloud verification, only when pose flagged
    # ====================================================================
    def _gemini_loop(self):
        url = ('https://generativelanguage.googleapis.com/v1beta/models/'
               'gemini-2.5-flash:generateContent?key=' + self.api_key)
        headers = {'Content-Type': 'application/json'}
        prompt = (
            'Analyze security camera. Return pure JSON, DO NOT wrap in ```json.\n'
            'DANGEROUS (is_suspicious=true) if: holding a weapon, break-in, '
            'covering face, attacking.\n'
            'SAFE (is_suspicious=false) if: walking, standing, sitting normally.\n'
            '{"is_suspicious": true/false, "reason": "short explanation in English"}'
        )

        last_person = False
        while rclpy.ok():
            time.sleep(0.2)
            with self._lock:
                person       = self._person
                pose_flagged = self._pose_flagged
                fb64         = self._best_jpg_b64

            person_just_appeared = person and not last_person
            last_person = person

            if not person:
                with self._lock:
                    self._suspicious = False
                    self._streak     = 0
                    self._reason     = 'No person detected'
                continue

            now = time.time()
            needs_check = (pose_flagged or person_just_appeared
                           or (now - self._last_gemini_t > self.recheck_interval))

            if not needs_check:
                with self._lock:
                    if not self._suspicious:
                        self._reason = 'Normal pose'
                continue

            if now - self._last_gemini_t < self.cooldown:
                continue

            if not fb64 or not self.api_key:
                with self._lock:
                    self._reason = 'Abnormal pose (no Gemini API key)'
                continue

            today_str = time.strftime('%Y-%m-%d')
            with self._lock:
                if today_str != self._api_date:
                    self._api_date        = today_str
                    self._api_calls_today = 0
                    self.get_logger().info('Daily API counter reset.')
                calls_today = self._api_calls_today

            if calls_today >= self.daily_limit:
                self.get_logger().warn(
                    f'Daily limit of {self.daily_limit} calls reached.',
                    throttle_duration_sec=60.0)
                with self._lock:
                    self._suspicious = pose_flagged
                    self._reason     = 'Abnormal pose (Gemini daily quota exceeded)'
                continue

            payload = {
                'contents': [{'parts': [
                    {'text': prompt},
                    {'inline_data': {'mime_type': 'image/jpeg', 'data': fb64}}
                ]}],
                'generationConfig': {
                    'temperature': 0.0,
                    'responseMimeType': 'application/json',
                    'responseSchema': {
                        'type': 'OBJECT',
                        'properties': {
                            'is_suspicious': {'type': 'BOOLEAN'},
                            'reason':        {'type': 'STRING'}
                        },
                        'required': ['is_suspicious', 'reason']
                    }
                },
                'safetySettings': [
                    {'category': 'HARM_CATEGORY_DANGEROUS_CONTENT',
                     'threshold': 'BLOCK_NONE'},
                    {'category': 'HARM_CATEGORY_HARASSMENT',
                     'threshold': 'BLOCK_NONE'}
                ]
            }

            try:
                resp = requests.post(url, headers=headers, json=payload, timeout=10)
                self._last_gemini_t = time.time()
                with self._lock:
                    self._api_calls_today += 1
                    calls_today = self._api_calls_today

                if resp.status_code != 200:
                    self.get_logger().warn(
                        f'Gemini HTTP {resp.status_code}: {resp.text[:200]}',
                        throttle_duration_sec=10.0)
                    continue

                candidates = resp.json().get('candidates', [])
                if not candidates:
                    self.get_logger().warn(
                        f'Gemini no candidates: {resp.json().get("promptFeedback", {})}',
                        throttle_duration_sec=10.0)
                    continue

                raw = candidates[0]['content']['parts'][0]['text'].strip()
                for prefix in ('```json', '```'):
                    if raw.startswith(prefix):
                        raw = raw[len(prefix):]
                if raw.endswith('```'):
                    raw = raw[:-3]

                data = json.loads(raw.strip())
                sus  = bool(data.get('is_suspicious', False))
                with self._lock:
                    self._streak     = (self._streak + 1) if sus else 0
                    self._suspicious = self._streak >= self.persistence
                    self._reason     = data.get('reason') or (
                        'Suspicious behaviour' if sus else 'Normal')
                self.get_logger().info(
                    f'Gemini [{calls_today}/{self.daily_limit}]: '
                    f'suspicious={sus} | {self._reason}')
            except Exception as ex:
                self.get_logger().warn(
                    f'Gemini error: {ex}', throttle_duration_sec=10.0)

    # ====================================================================
    # PUBLISH — every 0.5s
    # ====================================================================
    def _publish(self):
        with self._lock:
            payload = json.dumps({
                'is_suspicious':    self._suspicious,
                'person':           self._person,
                'pose_flagged':     self._pose_flagged,
                'reason':           self._reason,
                'api_calls_today':  self._api_calls_today
            })
        self.status_pub.publish(String(data=payload))


def main(args=None):
    rclpy.init(args=args)
    node = SecurityAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
