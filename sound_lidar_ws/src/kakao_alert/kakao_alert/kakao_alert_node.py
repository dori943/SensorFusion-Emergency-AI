#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""카카오톡 알림 노드.

emergency_fusion의 최종 판정을 받아 보호자에게 카카오톡 메시지를 보낸다.

구독:
  /emergency/fall_alert  (std_msgs/String, JSON)  fall_fusion_node
  /emergency/sos_alert   (std_msgs/String, JSON)  sos_fusion_node

주의 — level 필터가 반드시 필요하다:
  fall_fusion_node는 사건이 '종료'될 때도 같은 토픽으로 알림을 한 번
  발행한다(level="resolved", severity="info"). event_type만 보고
  판정하면 "fall_alert"에 'fall'이 들어 있으므로, 쓰러진 사람이 일어나
  회복했을 때 보호자에게 "낙상 감지되었습니다"가 발송된다.
  그래서 level을 화이트리스트로 걸러 confirmed/critical만 보낸다.
"""

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BASE_DIR = Path(__file__).resolve().parent

# 설정 파일 경로는 환경변수로 덮어쓸 수 있다.
#   __file__ 기준으로만 찾으면 --symlink-install 없이 빌드했을 때
#   install/ 밑을 보게 되고, `rm -rf install`로 재빌드하는 순간
#   access_token/refresh_token이 통째로 날아간다.
#   운영 시에는 워크스페이스 밖 경로를 지정하는 것을 권장한다.
#     export KAKAO_CONFIG=~/.config/kakao_alert/kakao_config.json
CONFIG_PATH = Path(
    os.environ.get("KAKAO_CONFIG", str(BASE_DIR / "kakao_config.json"))
).expanduser()

KAKAO_FRIEND_SEND_URL = "https://kapi.kakao.com/v1/api/talk/friends/message/default/send"
KAKAO_TOKEN_URL = "https://kauth.kakao.com/oauth/token"

# 이 레벨일 때만 카카오톡을 보낸다.
#   pending  : 근거 부족(0.70 구간) — 아직 확정 아님, 보내지 않는다
#   resolved : 회복/이탈로 종료 — 보내면 오히려 혼란
ALERT_LEVELS = {"confirmed", "critical"}


def load_config() -> Dict[str, Any]:
    if not CONFIG_PATH.exists():
        raise FileNotFoundError(
            f"Config file not found: {CONFIG_PATH}\n"
            "Copy kakao_config.example.json to kakao_config.json and fill in your values.\n"
            "Or set KAKAO_CONFIG to an absolute path."
        )
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_config(config: Dict[str, Any]) -> None:
    temp_path = CONFIG_PATH.with_suffix(".json.tmp")
    with temp_path.open("w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
    temp_path.replace(CONFIG_PATH)


class KakaoClient:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        kakao = config["kakao"]

        self.access_token = str(kakao.get("access_token", "")).strip()
        self.refresh_token = str(kakao.get("refresh_token", "")).strip()
        self.rest_api_key = str(kakao.get("rest_api_key", "")).strip()
        self.client_secret = str(kakao.get("client_secret", "")).strip()
        self.link_url = str(
            kakao.get("message_link_url", "https://developers.kakao.com")
        ).strip()

        self.receiver_uuids: List[str] = [
            str(x).strip()
            for x in kakao.get("receiver_uuids", [])
            if str(x).strip()
        ]

        if not self.access_token:
            raise ValueError("kakao.access_token is empty.")
        if not self.receiver_uuids:
            raise ValueError("kakao.receiver_uuids is empty.")
        if len(self.receiver_uuids) > 5:
            raise ValueError(
                "Kakao friend message API supports up to 5 receiver UUIDs per request."
            )
        if not self.link_url:
            raise ValueError("kakao.message_link_url is empty.")

    def _can_refresh(self) -> bool:
        return bool(self.refresh_token and self.rest_api_key)

    def refresh_access_token(self) -> bool:
        if not self._can_refresh():
            print(
                "[KAKAO] Cannot auto-refresh token. "
                "Set refresh_token and rest_api_key in kakao_config.json."
            )
            return False

        data = {
            "grant_type": "refresh_token",
            "client_id": self.rest_api_key,
            "refresh_token": self.refresh_token,
        }

        if self.client_secret:
            data["client_secret"] = self.client_secret

        try:
            response = requests.post(KAKAO_TOKEN_URL, data=data, timeout=10)
        except requests.RequestException as e:
            print(f"[KAKAO] Token refresh request failed: {e}")
            return False

        if response.status_code != 200:
            print(
                f"[KAKAO] Token refresh failed | "
                f"HTTP {response.status_code} | {response.text}"
            )
            return False

        result = response.json()
        new_access_token = str(result.get("access_token", "")).strip()

        if not new_access_token:
            print(f"[KAKAO] No access_token in refresh response: {result}")
            return False

        self.access_token = new_access_token
        self.config["kakao"]["access_token"] = new_access_token

        new_refresh_token = str(result.get("refresh_token", "")).strip()
        if new_refresh_token:
            self.refresh_token = new_refresh_token
            self.config["kakao"]["refresh_token"] = new_refresh_token

        save_config(self.config)

        expires_in = result.get("expires_in")
        print(
            "[KAKAO] Access token refreshed successfully"
            + (f" | expires_in={expires_in}s" if expires_in is not None else "")
        )
        return True

    def _send_once(self, text: str) -> requests.Response:
        text = text[:200]

        template_object = {
            "object_type": "text",
            "text": text,
            "link": {
                "web_url": self.link_url,
                "mobile_web_url": self.link_url,
            },
        }

        headers = {
            "Authorization": f"Bearer {self.access_token}",
            "Content-Type": "application/x-www-form-urlencoded;charset=utf-8",
        }

        data = {
            "receiver_uuids": json.dumps(
                self.receiver_uuids,
                ensure_ascii=False,
                separators=(",", ":"),
            ),
            "template_object": json.dumps(
                template_object,
                ensure_ascii=False,
                separators=(",", ":"),
            ),
        }

        return requests.post(
            KAKAO_FRIEND_SEND_URL,
            headers=headers,
            data=data,
            timeout=10,
        )

    def send(self, text: str) -> bool:
        try:
            response = self._send_once(text)
        except requests.RequestException as e:
            print(f"[KAKAO] Message request failed: {e}")
            return False

        if response.status_code == 401 and self._can_refresh():
            print("[KAKAO] Access token rejected. Refreshing and retrying once...")
            if self.refresh_access_token():
                try:
                    response = self._send_once(text)
                except requests.RequestException as e:
                    print(f"[KAKAO] Retry request failed: {e}")
                    return False

        if response.status_code != 200:
            print(
                f"[KAKAO] Message send failed | "
                f"HTTP {response.status_code} | {response.text}"
            )
            return False

        try:
            result = response.json()
        except ValueError:
            print(f"[KAKAO] Invalid response: {response.text}")
            return False

        successful = result.get("successful_receiver_uuids", [])
        failure_info = result.get("failure_info", [])

        if successful:
            print(f"[KAKAO SENT] {text} | receivers={len(successful)}")
        else:
            print(f"[KAKAO] No successful_receiver_uuids | response={result}")

        if failure_info:
            print(f"[KAKAO] Partial failure: {failure_info}")

        return bool(successful)


# ── 페이로드 해석 ────────────────────────────────────────────
def parse_alert(raw: str) -> Optional[Dict[str, Any]]:
    """fusion 노드의 알림 JSON을 딕셔너리로. JSON이 아니면 None."""
    try:
        obj = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None


def extract_value(raw: str) -> str:
    """JSON이 아닌 단순 문자열 입력을 위한 fallback 파서."""
    text = raw.strip()
    if not text:
        return ""

    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        return text

    if isinstance(obj, str):
        return obj.strip()
    if isinstance(obj, bool):
        return "true" if obj else "false"
    if isinstance(obj, (int, float)):
        return str(obj)

    if isinstance(obj, dict):
        for key in ("state", "status", "result", "verdict", "event_type", "type"):
            if key in obj:
                value = obj[key]
                if isinstance(value, bool):
                    return "true" if value else "false"
                return str(value).strip()

    return text


def normalize_combined_state(raw: str) -> Optional[str]:
    value = extract_value(raw).strip().lower()
    if not value:
        return None

    compact = value.replace("-", "_").replace(" ", "_")

    normal_tokens = {
        "normal", "safe", "none", "idle", "clear",
        "정상", "안전",
    }
    fall_tokens = {
        "fall", "fallen", "fall_detected", "fall_confirmed",
        "낙상", "낙상감지", "낙상_감지",
    }
    sos_tokens = {
        "sos", "sos_detected", "sos_confirmed",
        "구조요청", "구조_요청",
    }

    if compact in normal_tokens:
        return "normal"
    if compact in fall_tokens:
        return "fall"
    if compact in sos_tokens:
        return "sos"

    if "sos" in compact:
        return "sos"
    if "fall" in compact or "낙상" in compact:
        return "fall"

    return None


def separate_topic_is_detected(raw: str, event_type: str) -> bool:
    value = extract_value(raw).strip().lower()
    compact = value.replace("-", "_").replace(" ", "_")

    false_tokens = {
        "", "0", "false", "normal", "safe", "none", "idle", "clear",
        "not_detected", "정상", "안전",
    }
    true_tokens = {
        "1", "true", "detected", "confirmed", "yes",
    }

    if compact in false_tokens:
        return False
    if compact in true_tokens:
        return True

    if event_type == "fall":
        return "fall" in compact or "낙상" in compact
    if event_type == "sos":
        return "sos" in compact or "구조" in compact

    return False


class KakaoAlertNode(Node):
    def __init__(self):
        super().__init__("kakao_alert_node")

        self.config = load_config()
        self.kakao = KakaoClient(self.config)

        ros_cfg = self.config.get("ros", {})
        message_cfg = self.config.get("messages", {})

        self.combined_topic = str(
            ros_cfg.get("combined_result_topic", "")
        ).strip()
        self.fall_topic = str(
            ros_cfg.get("fall_result_topic", "/emergency/fall_alert")
        ).strip()
        self.sos_topic = str(
            ros_cfg.get("sos_result_topic", "/emergency/sos_alert")
        ).strip()

        self.cooldown_sec = float(ros_cfg.get("cooldown_sec", 30.0))
        # 위급(critical)은 더 자주 보내고 싶을 때 사용. 기본은 동일.
        self.critical_cooldown_sec = float(
            ros_cfg.get("critical_cooldown_sec", self.cooldown_sec)
        )

        self.fall_message = str(
            message_cfg.get("fall", "낙상이 감지되었습니다. 확인이 필요합니다.")
        )
        self.fall_critical_message = str(
            message_cfg.get(
                "fall_critical",
                "위급: 낙상 후 일어나지 못하고 있습니다. 즉시 확인해 주세요.",
            )
        )
        self.sos_message = str(
            message_cfg.get("sos", "구조 요청 신호가 감지되었습니다.")
        )

        self.last_combined_state: Optional[str] = None
        self.last_sent_time = {"fall": 0.0, "sos": 0.0}
        # 같은 사건에 대해 레벨이 올라갔을 때는 쿨다운을 무시하고
        # 한 번 더 보낸다(확정 → 위급은 상황이 악화된 것이므로).
        self.last_level = {"fall": None, "sos": None}

        subscriptions = []

        if self.combined_topic:
            subscriptions.append(
                self.create_subscription(
                    String, self.combined_topic, self._combined_callback, 10
                )
            )

        if self.fall_topic:
            subscriptions.append(
                self.create_subscription(
                    String, self.fall_topic, self._fall_callback, 10
                )
            )

        if self.sos_topic:
            subscriptions.append(
                self.create_subscription(
                    String, self.sos_topic, self._sos_callback, 10
                )
            )

        self._subs = subscriptions

        if not subscriptions:
            raise ValueError(
                "No ROS result topic configured. "
                "Set combined_result_topic or fall_result_topic/sos_result_topic."
            )

        topics = [
            t for t in (self.combined_topic, self.fall_topic, self.sos_topic) if t
        ]

        self.get_logger().info(
            "Kakao alert node started | topics=" + ", ".join(topics)
            + f" | 발송 레벨={sorted(ALERT_LEVELS)}"
            + f" | cooldown={self.cooldown_sec}s"
            + f" (critical={self.critical_cooldown_sec}s)"
        )

    # ── 발송 제어 ───────────────────────────────────────────
    def _can_send_now(self, event_type: str, level: Optional[str]) -> bool:
        # 레벨이 올라간 경우(confirmed → critical)는 쿨다운을 건너뛴다.
        # 상황이 악화됐는데 30초를 기다리면 알림의 의미가 없다.
        if level == "critical" and self.last_level.get(event_type) != "critical":
            return True

        now = time.monotonic()
        cooldown = (
            self.critical_cooldown_sec if level == "critical" else self.cooldown_sec
        )
        return (now - self.last_sent_time[event_type]) >= cooldown

    def _send_event(self, event_type: str, level: Optional[str] = None) -> None:
        if not self._can_send_now(event_type, level):
            self.get_logger().info(
                f"{event_type.upper()} notification suppressed by cooldown "
                f"(level={level})."
            )
            return

        if event_type == "fall":
            text = (
                self.fall_critical_message
                if level == "critical"
                else self.fall_message
            )
        elif event_type == "sos":
            text = self.sos_message
        else:
            return

        if self.kakao.send(text):
            self.last_sent_time[event_type] = time.monotonic()
            self.last_level[event_type] = level

    # ── 콜백 ────────────────────────────────────────────────
    def _handle_alert(self, msg: String, event_type: str) -> None:
        """fusion 노드의 JSON 알림 처리.

        level 화이트리스트로 거르는 것이 핵심이다. event_type 문자열만
        보면 종료 알림(level="resolved")까지 '낙상 감지'로 발송된다.
        """
        obj = parse_alert(msg.data)

        if obj is not None and "level" in obj:
            level = str(obj.get("level", "")).strip().lower()
            if level not in ALERT_LEVELS:
                self.get_logger().info(
                    f"{event_type.upper()} alert ignored (level={level}) "
                    f"— 확정 전이거나 이미 종료된 사건입니다."
                )
                # 종료되면 다음 사건을 위해 레벨 기록을 초기화한다.
                if level == "resolved":
                    self.last_level[event_type] = None
                return

            case_id = obj.get("case_id")
            reason = obj.get("reason")
            self.get_logger().warning(
                f"{event_type.upper()} verdict | level={level} "
                f"case={case_id} conf={obj.get('confidence')} | {reason}"
            )
            self._send_event(event_type, level)
            return

        # JSON이 아니거나 level이 없는 단순 문자열 입력 (구형 호환)
        if separate_topic_is_detected(msg.data, event_type):
            self.get_logger().info(
                f"{event_type.upper()} verdict received (plain): {msg.data}"
            )
            self._send_event(event_type, None)

    def _fall_callback(self, msg: String) -> None:
        self._handle_alert(msg, "fall")

    def _sos_callback(self, msg: String) -> None:
        self._handle_alert(msg, "sos")

    def _combined_callback(self, msg: String) -> None:
        state = normalize_combined_state(msg.data)

        if state is None:
            self.get_logger().warning(f"Unknown combined result: {msg.data!r}")
            return

        if state == self.last_combined_state:
            return

        previous = self.last_combined_state
        self.last_combined_state = state

        self.get_logger().info(f"Final state changed: {previous} -> {state}")

        if state == "fall":
            self._send_event("fall", None)
        elif state == "sos":
            self._send_event("sos", None)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = KakaoAlertNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, ValueError) as e:
        print(f"[ERROR] {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()