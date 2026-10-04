import csv
import math
import os
import serial
import struct
import time
from datetime import datetime

# =========================================================
# Radar / serial settings
# =========================================================

# 현재 PC에서 확인된 CP2105 포트 구성
# Enhanced COM Port -> CLI / config
# Standard COM Port -> DATA
CLI_PORT = "COM5"
CLI_BAUD = 115200

DATA_PORT = "COM3"
DATA_BAUD = 921600

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CFG_FILE = os.path.join(BASE_DIR, "profile_3d_aop.cfg")

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

HEADER_SIZE = 40


# =========================================================
# Robot-side risk parameters (first-test values)
# =========================================================

# TI mmWave demo의 Doppler를 현재 프로젝트에서는
# 음수 = 레이더(로봇) 쪽으로 접근하는 것으로 사용한다.
# 실험 결과 부호가 반대로 보이면 -1.0 -> +1.0 으로 바꾸면 된다.
APPROACH_DOPPLER_SIGN = -1.0

# Doppler deadband: 이보다 작은 값은 정지/노이즈로 취급
DOPPLER_DEADBAND_MPS = 0.08

# 현재 로봇 주변에서 위험 후보로 볼 거리 범위
MIN_RANGE_M = 0.15
MAX_RANGE_M = 1.50

# Side info가 존재할 때만 적용한다.
# SNR이 None이면 첫 테스트에서는 버리지 않는다.
MIN_SNR_DB = 10.0

# TTC 기반 위험 단계 (초)
TTC_WATCH_S = 3.0
TTC_WARNING_S = 2.0
TTC_AVOID_S = 1.2
TTC_STOP_S = 0.6

# 매우 가까운 접근 물체에 대한 추가 안전 기준
EMERGENCY_DISTANCE_M = 0.20


# =========================================================
# Radar configuration
# =========================================================

def configure_radar():
    print("[RADAR] Opening CLI port...")

    cli = serial.Serial(
        CLI_PORT,
        CLI_BAUD,
        timeout=1
    )

    time.sleep(1)

    # 기존 동작 정지
    cli.write(b"sensorStop\n")
    time.sleep(0.2)

    print("[RADAR] Sending cfg...")
    print("[CFG]", CFG_FILE)

    with open(CFG_FILE, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            if line.startswith("%"):
                continue

            # 마지막에 한 번만 sensorStart
            if line.startswith("sensorStart"):
                continue

            cli.write((line + "\n").encode())
            print(">", line)
            time.sleep(0.03)

    cli.write(b"sensorStart\n")

    print("> sensorStart")
    print("[RADAR] Configuration complete")

    time.sleep(0.5)
    cli.close()


# =========================================================
# Frame parser
# =========================================================

def parse_frame(packet):
    if len(packet) < HEADER_SIZE:
        return None

    try:
        header = struct.unpack_from(
            "<8s8I",
            packet,
            0
        )
    except struct.error:
        return None

    magic = header[0]

    if magic != MAGIC_WORD:
        return None

    frame_number = header[4]
    num_detected_obj = header[6]
    num_tlvs = header[7]

    offset = HEADER_SIZE

    points = []
    side_info = []

    # =====================================================
    # TLV parsing
    # =====================================================
    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        tlv_type, tlv_length = struct.unpack_from(
            "<II",
            packet,
            offset
        )

        offset += 8

        payload_start = offset
        payload_end = offset + tlv_length

        if payload_end > len(packet):
            break

        # -------------------------------------------------
        # TLV 1 : Detected Points
        # x, y, z, Doppler
        # -------------------------------------------------
        if tlv_type == TLV_DETECTED_POINTS:
            point_size = 16
            available_points = tlv_length // point_size

            count = min(
                num_detected_obj,
                available_points
            )

            for i in range(count):
                p_offset = payload_start + i * point_size

                x, y, z, doppler = struct.unpack_from(
                    "<ffff",
                    packet,
                    p_offset
                )

                points.append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "doppler": doppler
                })

        # -------------------------------------------------
        # TLV 7 : Side Info
        # SNR / Noise
        # 단위 = 0.1 dB
        # -------------------------------------------------
        elif tlv_type == TLV_SIDE_INFO:
            side_size = 4
            available_side = tlv_length // side_size

            count = min(
                num_detected_obj,
                available_side
            )

            for i in range(count):
                s_offset = payload_start + i * side_size

                snr_raw, noise_raw = struct.unpack_from(
                    "<HH",
                    packet,
                    s_offset
                )

                side_info.append({
                    "snr": snr_raw / 10.0,
                    "noise": noise_raw / 10.0
                })

        offset = payload_end

    # =====================================================
    # Point + Side Info 결합
    # =====================================================
    for i in range(len(points)):
        if i < len(side_info):
            points[i]["snr"] = side_info[i]["snr"]
            points[i]["noise"] = side_info[i]["noise"]
        else:
            points[i]["snr"] = None
            points[i]["noise"] = None

    return {
        "frame": frame_number,
        "num_obj": num_detected_obj,
        "num_tlvs": num_tlvs,
        "points": points
    }


# =========================================================
# Robot-side point processing
# =========================================================

def classify_risk(distance_m, approach_speed_mps, ttc_s, is_candidate):
    """현재는 모터 명령을 내리지 않고 위험도만 분류한다."""

    if not is_candidate or approach_speed_mps <= 0.0 or ttc_s is None:
        return "SAFE"

    if distance_m <= EMERGENCY_DISTANCE_M or ttc_s <= TTC_STOP_S:
        return "STOP"

    if ttc_s <= TTC_AVOID_S:
        return "AVOID"

    if ttc_s <= TTC_WARNING_S:
        return "WARNING"

    if ttc_s <= TTC_WATCH_S:
        return "WATCH"

    return "SAFE"


def calculate_point_metrics(point):
    """
    레이더 포인트 1개를 로봇 관점의 거리/방향/상대 접근속도/TTC로 변환한다.

    angle_deg는 x-y 평면에서 y축 정면을 0도로 본 값이다.
    실제 좌/우 부호는 레이더 장착 방향에 따라 현장에서 한 번 확인한다.
    """

    x = point["x"]
    y = point["y"]
    z = point["z"]
    doppler = point["doppler"]
    snr = point["snr"]

    horizontal_distance_m = math.hypot(x, y)
    distance_m = math.sqrt(x * x + y * y + z * z)

    # y축 정면 기준 각도
    angle_deg = math.degrees(math.atan2(x, y))

    # 상대 방사속도를 '접근속도 양수' 형태로 변환
    signed_approach_mps = APPROACH_DOPPLER_SIGN * doppler

    if signed_approach_mps > DOPPLER_DEADBAND_MPS:
        approach_speed_mps = signed_approach_mps
    else:
        approach_speed_mps = 0.0

    if approach_speed_mps > 0.0:
        ttc_s = distance_m / approach_speed_mps
    else:
        ttc_s = None

    range_ok = MIN_RANGE_M <= distance_m <= MAX_RANGE_M
    snr_ok = snr is None or snr >= MIN_SNR_DB
    motion_ok = approach_speed_mps > 0.0

    is_candidate = range_ok and snr_ok and motion_ok

    risk = classify_risk(
        distance_m,
        approach_speed_mps,
        ttc_s,
        is_candidate
    )

    return {
        "distance_m": distance_m,
        "horizontal_distance_m": horizontal_distance_m,
        "angle_deg": angle_deg,
        "approach_speed_mps": approach_speed_mps,
        "ttc_s": ttc_s,
        "is_candidate": is_candidate,
        "risk": risk
    }


# =========================================================
# Main
# =========================================================

def main():
    # -----------------------------------------------------
    # 1. Radar cfg 자동 전송
    # -----------------------------------------------------
    configure_radar()

    # -----------------------------------------------------
    # 2. Data port open
    # -----------------------------------------------------
    print("[RADAR] Opening data port...")

    data_serial = serial.Serial(
        DATA_PORT,
        DATA_BAUD,
        timeout=0.1
    )

    time.sleep(0.5)
    data_serial.reset_input_buffer()

    # -----------------------------------------------------
    # 3. CSV
    # -----------------------------------------------------
    filename = datetime.now().strftime(
        "radar_robot_log_%Y%m%d_%H%M%S.csv"
    )
    csv_path = os.path.join(BASE_DIR, filename)

    csv_file = open(
        csv_path,
        "w",
        newline=""
    )

    writer = csv.writer(csv_file)

    writer.writerow([
        "timestamp_s",
        "frame_id",
        "point_id",
        "x_m",
        "y_m",
        "z_m",
        "distance_m",
        "horizontal_distance_m",
        "angle_deg",
        "doppler_mps",
        "approach_speed_mps",
        "ttc_s",
        "risk",
        "is_candidate",
        "snr_db",
        "noise_db"
    ])

    print("[CSV]", csv_path)
    print("[RADAR] Receiving...")
    print(
        f"[FILTER] range={MIN_RANGE_M:.2f}~{MAX_RANGE_M:.2f} m | "
        f"deadband={DOPPLER_DEADBAND_MPS:.2f} m/s | "
        f"min SNR={MIN_SNR_DB:.1f} dB"
    )
    print("Ctrl+C to stop")

    buffer = bytearray()
    start_time = time.time()

    try:
        while True:
            # =================================================
            # Serial read
            # =================================================
            available = data_serial.in_waiting

            if available > 0:
                buffer.extend(
                    data_serial.read(available)
                )

            # =================================================
            # Magic word 찾기
            # =================================================
            magic_index = buffer.find(MAGIC_WORD)

            if magic_index == -1:
                # 너무 커지지 않도록 magic 길이 일부만 보존
                if len(buffer) > 4096:
                    buffer = buffer[-7:]
                continue

            # magic 앞 쓰레기 데이터 제거
            if magic_index > 0:
                del buffer[:magic_index]

            # header가 아직 다 안 들어옴
            if len(buffer) < HEADER_SIZE:
                continue

            # =================================================
            # Total Packet Length
            # =================================================
            total_packet_len = struct.unpack_from(
                "<I",
                buffer,
                12
            )[0]

            # 비정상 packet 방어
            if (
                total_packet_len < HEADER_SIZE
                or total_packet_len > 65536
            ):
                del buffer[0]
                continue

            # packet 전체가 아직 안 들어옴
            if len(buffer) < total_packet_len:
                continue

            packet = bytes(
                buffer[:total_packet_len]
            )

            del buffer[:total_packet_len]

            # =================================================
            # Frame parsing
            # =================================================
            result = parse_frame(packet)

            if result is None:
                continue

            frame_id = result["frame"]
            points = result["points"]
            elapsed = time.time() - start_time

            processed_points = []

            for point_id, p in enumerate(points):
                metrics = calculate_point_metrics(p)
                processed_points.append((point_id, p, metrics))

            candidates = [
                item
                for item in processed_points
                if item[2]["is_candidate"] and item[2]["ttc_s"] is not None
            ]

            # TTC가 가장 작은 포인트 = 현재 프레임의 대표 위험 포인트
            target = (
                min(candidates, key=lambda item: item[2]["ttc_s"])
                if candidates
                else None
            )

            print(
                f"\nFrame {frame_id:6d} | "
                f"Points {len(points):3d} | "
                f"Candidates {len(candidates):2d} | "
                f"TLV {result['num_tlvs']}"
            )

            # =================================================
            # Point 출력 + CSV
            # =================================================
            for point_id, p, m in processed_points:
                snr = p["snr"]
                noise = p["noise"]

                snr_text = (
                    f"{snr:.1f}"
                    if snr is not None
                    else "N/A"
                )

                ttc_text = (
                    f"{m['ttc_s']:.2f}s"
                    if m["ttc_s"] is not None
                    else "N/A"
                )

                candidate_mark = "*" if m["is_candidate"] else " "

                print(
                    f" {candidate_mark}P{point_id:02d} "
                    f"Dist={m['distance_m']:.2f}m "
                    f"Ang={m['angle_deg']:+6.1f}deg "
                    f"Dop={p['doppler']:+.2f}m/s "
                    f"Approach={m['approach_speed_mps']:.2f}m/s "
                    f"TTC={ttc_text:>6} "
                    f"Risk={m['risk']:<7} "
                    f"SNR={snr_text}dB"
                )

                writer.writerow([
                    round(elapsed, 3),
                    frame_id,
                    point_id,
                    p["x"],
                    p["y"],
                    p["z"],
                    m["distance_m"],
                    m["horizontal_distance_m"],
                    m["angle_deg"],
                    p["doppler"],
                    m["approach_speed_mps"],
                    m["ttc_s"] if m["ttc_s"] is not None else "",
                    m["risk"],
                    int(m["is_candidate"]),
                    snr if snr is not None else "",
                    noise if noise is not None else ""
                ])

            # =================================================
            # Frame 대표 위험 포인트 출력
            # =================================================
            if target is not None:
                target_id, target_point, target_metrics = target

                print(
                    "[TARGET] "
                    f"P{target_id:02d} | "
                    f"Dist={target_metrics['distance_m']:.2f}m | "
                    f"Angle={target_metrics['angle_deg']:+.1f}deg | "
                    f"Approach={target_metrics['approach_speed_mps']:.2f}m/s | "
                    f"TTC={target_metrics['ttc_s']:.2f}s | "
                    f"Risk={target_metrics['risk']}"
                )
            else:
                print("[TARGET] none")

            csv_file.flush()

    except KeyboardInterrupt:
        print("\n[RADAR] Stopped")

    finally:
        csv_file.close()
        data_serial.close()

        print("[CSV] Saved:", csv_path)


# =========================================================
# Run
# =========================================================

if __name__ == "__main__":
    main()
