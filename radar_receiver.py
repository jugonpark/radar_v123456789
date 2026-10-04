import serial
import struct
import time
import csv
import os
from datetime import datetime

# =========================================================
# Radar settings
# =========================================================

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

    version = header[1]
    total_packet_len = header[2]
    platform = header[3]
    frame_number = header[4]
    time_cpu_cycles = header[5]
    num_detected_obj = header[6]
    num_tlvs = header[7]
    subframe_number = header[8]

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
        "radar_log_%Y%m%d_%H%M%S.csv"
    )

    csv_file = open(
        filename,
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
        "doppler_mps",
        "snr_db",
        "noise_db"
    ])

    print("[CSV]", filename)
    print("[RADAR] Receiving...")
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
                or
                total_packet_len > 65536
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

            print(
                f"Frame {frame_id:6d} | "
                f"Points {len(points):3d} | "
                f"TLV {result['num_tlvs']}"
            )

            # =================================================
            # Point 출력 + CSV
            # =================================================

            for point_id, p in enumerate(points):

                snr = p["snr"]
                noise = p["noise"]

                snr_text = (
                    f"{snr:.1f}"
                    if snr is not None
                    else "N/A"
                )

                print(
                    f"  P{point_id:02d} "
                    f"x={p['x']:+.3f} "
                    f"y={p['y']:+.3f} "
                    f"z={p['z']:+.3f} "
                    f"v={p['doppler']:+.3f} m/s "
                    f"SNR={snr_text} dB"
                )

                writer.writerow([
                    round(elapsed, 3),
                    frame_id,
                    point_id,
                    p["x"],
                    p["y"],
                    p["z"],
                    p["doppler"],
                    snr if snr is not None else "",
                    noise if noise is not None else ""
                ])

            csv_file.flush()

    except KeyboardInterrupt:

        print("\n[RADAR] Stopped")

    finally:

        csv_file.close()
        data_serial.close()

        print("[CSV] Saved:", filename)


# =========================================================
# Run
# =========================================================

if __name__ == "__main__":
    main()