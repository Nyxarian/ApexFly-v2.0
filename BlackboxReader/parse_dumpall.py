# -*- coding: utf-8 -*-
"""一次性整理 dumpall.bin：拆新格式场次，统计老/垃圾数据。"""
from __future__ import annotations

import os
import struct
from collections import Counter

SRC_DIR = r"F:\Zhuomian\ApexFly v2.0\BlackboxReader\exports\blackbox_20260823_125133"
KEEP_DIR = os.path.join(SRC_DIR, "keep_v2")
BIN_PATH = os.path.join(SRC_DIR, "dumpall.bin")

MAGIC = 0x424C5832  # BLX2 little-endian 32 58 4C 42
TYPE_BEGIN = 1
TYPE_SAMPLE = 2
TYPE_END = 3
REC = 64
CRC_OFF = 60

PID_NAMES = [
    "roll_outer_kp", "roll_outer_ki", "roll_outer_kd",
    "roll_inner_kp", "roll_inner_ki", "roll_inner_kd",
    "pitch_outer_kp", "pitch_outer_ki", "pitch_outer_kd",
    "pitch_inner_kp", "pitch_inner_ki", "pitch_inner_kd",
    "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_kd",
    "roll_max_rate_dps", "roll_max_corr_us",
    "pitch_max_rate_dps", "pitch_max_corr_us",
    "yaw_max_rate_dps", "yaw_max_corr_us",
    "roll_trim_deg", "pitch_trim_deg",
    "yaw_angle_kp", "yaw_angle_ki", "yaw_angle_kd",
]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def classify(rec: bytes) -> str:
    if len(rec) < REC:
        return "truncated"
    magic = u32(rec, 0)
    if magic == 0xFFFFFFFF:
        return "erased"
    if magic != MAGIC:
        return "old_or_garbage"
    if crc16(rec[:CRC_OFF]) != u16(rec, CRC_OFF):
        return "blx2_bad_crc"
    rec_type = rec[4]
    if rec_type == TYPE_BEGIN:
        return "begin"
    if rec_type == TYPE_SAMPLE:
        return "sample"
    if rec_type == TYPE_END:
        return "end"
    return "blx2_unknown_type"


def pid_values(rec: bytes) -> list[float]:
    return [i16(rec, 8 + index * 2) / 100.0 for index in range(26)]


def sample_row(rec: bytes) -> list:
    return [
        u32(rec, 8),
        i16(rec, 12) / 100.0,
        i16(rec, 14) / 100.0,
        i16(rec, 16) / 100.0,
        i16(rec, 18) / 100.0,
        i16(rec, 20) / 100.0,
        i16(rec, 22) / 100.0,
        i16(rec, 24),
        i16(rec, 26),
        i16(rec, 28),
        u16(rec, 30),
        u16(rec, 32),
        u16(rec, 34),
        u16(rec, 36),
        u16(rec, 38),
        rec[40],
        rec[41],
    ]


def write_session(session: dict) -> None:
    sid = session["id"]
    stem = os.path.join(KEEP_DIR, f"session_{sid:03d}")
    header = ",".join(
        [
            "t_ms", "roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate",
            "roll_corr", "pitch_corr", "yaw_corr", "thr", "m1", "m2", "m3", "m4", "armed", "conn",
        ]
    )
    pid_text = " ".join(f"{index}={value:.2f}" for index, value in enumerate(session["pid"]))
    with open(stem + ".csv", "w", encoding="utf-8", newline="\n") as handle:
        handle.write(f"# session={sid} t0_utc=000000\n")
        handle.write(f"# pid {pid_text}\n")
        handle.write(header + "\n")
        for row in session["rows"]:
            handle.write(
                f"{row[0]},{row[1]:.2f},{row[2]:.2f},{row[3]:.2f},{row[4]:.2f},{row[5]:.2f},{row[6]:.2f},"
                f"{row[7]},{row[8]},{row[9]},{row[10]},{row[11]},{row[12]},{row[13]},{row[14]},{row[15]},{row[16]}\n"
            )
    with open(stem + ".vofa.csv", "w", encoding="utf-8", newline="\n") as handle:
        for row in session["rows"]:
            handle.write(
                f"{row[0]},{row[1]:.2f},{row[2]:.2f},{row[3]:.2f},{row[4]:.2f},{row[5]:.2f},{row[6]:.2f},"
                f"{row[7]},{row[8]},{row[9]},{row[10]},{row[11]},{row[12]},{row[13]},{row[14]},{row[15]},{row[16]}\n"
            )
    with open(stem + ".pid.csv", "w", encoding="utf-8", newline="\n") as handle:
        handle.write("id,name,value\n")
        for index, name in enumerate(PID_NAMES):
            handle.write(f"{index},{name},{session['pid'][index]:.4f}\n")


def load_by_address(raw_path: str) -> tuple[int, int, bytearray]:
    """按 dumpall 文本里的 Flash 地址把字节放回原位，避免从记录中间拼接。"""
    mapping = {}
    with open(raw_path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            try:
                addr = int(parts[0], 16)
            except ValueError:
                continue
            for index, item in enumerate(parts[1:]):
                try:
                    mapping[addr + index] = int(item, 16)
                except ValueError:
                    break
    if not mapping:
        raise RuntimeError("raw 文本里没有十六进制行")
    start = min(mapping)
    end = max(mapping) + 1
    blob = bytearray(end - start)
    for addr, value in mapping.items():
        blob[addr - start] = value
    return start, end, blob


def align_records(start_addr: int, blob: bytes) -> tuple[int, bytes]:
    """找到第一条 BLX2，按 64 字节边界切开。"""
    magic = b"\x32\x58\x4C\x42"
    index = blob.find(magic)
    if index < 0:
        return start_addr, blob
    return start_addr + index, blob[index:]


def main() -> None:
    os.makedirs(KEEP_DIR, exist_ok=True)
    raw_path = os.path.join(SRC_DIR, "dumpall_raw.txt")
    start_addr, end_addr, addressed = load_by_address(raw_path)
    aligned_addr, blob = align_records(start_addr, bytes(addressed))
    leftover = len(blob) % REC
    blob = blob[: len(blob) - leftover]
    counts = Counter()
    sessions = []
    current = None
    orphan_rows = []
    print(f"raw_span=0x{start_addr:08X}-0x{end_addr:08X} aligned=0x{aligned_addr:08X} rec={len(blob)//REC} leftover={leftover}")
    for offset in range(0, len(blob), REC):
        rec = blob[offset:offset + REC]
        kind = classify(rec)
        counts[kind] += 1
        if kind == "begin":
            if current is not None:
                current["complete"] = False
                sessions.append(current)
            current = {
                "id": u16(rec, 6),
                "pid": pid_values(rec),
                "rows": [],
                "complete": False,
                "end_samples": None,
                "end_t_ms": None,
                "file_offset": offset,
            }
        elif kind == "sample":
            row = sample_row(rec)
            if current is None:
                orphan_rows.append(row)
            else:
                current["rows"].append(row)
        elif kind == "end":
            if current is None:
                counts["end_without_begin"] += 1
            else:
                current["complete"] = current["id"] == u16(rec, 6)
                current["end_samples"] = u16(rec, 12)
                current["end_t_ms"] = u32(rec, 8)
                sessions.append(current)
                current = None
    if current is not None:
        current["complete"] = False
        sessions.append(current)

    # 从 dumpall_raw 第一行取起始地址
    start_addr = 0x0043DCD0
    for session in sessions:
        write_session(session)

    if orphan_rows:
        fake = {
            "id": 0,
            "pid": [0.0] * 26,
            "rows": orphan_rows,
            "complete": False,
            "end_samples": None,
            "end_t_ms": orphan_rows[-1][0] if orphan_rows else None,
            "file_offset": 0,
        }
        write_session(fake)
        sessions.insert(0, fake)

    lines = []
    lines.append("# dumpall 整理")
    lines.append("")
    lines.append(f"来源: `{raw_path}`")
    lines.append(f"地址范围: 0x{start_addr:08X} .. 0x{end_addr:08X}")
    lines.append(f"对齐后: 0x{aligned_addr:08X}，完整记录 {len(blob)//REC} 条，尾巴 {leftover} 字节丢掉")
    lines.append("串口原文没有 `[BB] dumpall start/done`，开头地址 0x0043DCD0（日志区从 0x004000 起），末尾一行被截断。")
    lines.append("也就是说：**Flash 开头大约 0x3DCD0 字节（约 249KB / 3900 条）这次没导出来。**")
    lines.append("")
    lines.append("## 记录分类")
    lines.append("")
    lines.append("| 类型 | 条数 | 说明 |")
    lines.append("|---|---:|---|")
    names = {
        "begin": "新格式场次头 BLX2",
        "sample": "新格式样本",
        "end": "新格式场次尾",
        "old_or_garbage": "不是 BLX2，老数据或垃圾",
        "blx2_bad_crc": "魔数对但 CRC 错",
        "erased": "全 FF 空区",
        "truncated": "不足 64 字节",
        "blx2_unknown_type": "BLX2 未知类型",
        "end_without_begin": "场次尾没有对应头",
    }
    for key, label in names.items():
        if counts[key]:
            lines.append(f"| {label} | {counts[key]} | `{key}` |")
    lines.append("")
    lines.append("## 本文件里拆出的新场次")
    lines.append("")
    lines.append("| 场次 | 样本 | 时长 ms | 完整 | Roll外环Kp | Pitch外环Kp | 内环Kp | 油门最大 |")
    lines.append("|---|---:|---:|---|---:|---:|---|---:|")
    for session in sessions:
        rows = session["rows"]
        thr_max = max((row[10] for row in rows), default=1000)
        inner = f"{session['pid'][3]:.2f}/{session['pid'][9]:.2f}"
        complete = "是" if session["complete"] else "否"
        t_ms = session["end_t_ms"] if session["end_t_ms"] is not None else (rows[-1][0] if rows else 0)
        lines.append(
            f"| {session['id']} | {len(rows)} | {t_ms} | {complete} | {session['pid'][0]:.2f} | {session['pid'][6]:.2f} | {inner} | {thr_max} |"
        )
    lines.append("")
    lines.append("场次号不连续，是 Flash 里以前的空洞/老数据把编号推上去了。")
    lines.append("没有一场 PID 是 2.30 / 1.50；这份里看到的外环要么 0.30 要么 1.00，内环仍是 0。")
    lines.append("")
    lines.append("CSV / VOFA / PID 已写到本目录 `keep_v2/`。")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(KEEP_DIR, "INDEX.md"), "w", encoding="utf-8") as handle:
        handle.write(text)
    print(text)
    print("counts", dict(counts))
    print("sessions", [(s["id"], len(s["rows"]), s["complete"], s["pid"][0], s["pid"][6]) for s in sessions])


if __name__ == "__main__":
    main()
