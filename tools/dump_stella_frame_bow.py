#!/usr/bin/env python3
"""Build + run the stella_port module-2 reference tool
(stella_port/reference_tools/dump_frame_bow.cc) on fr1_xyz/fr1_desk, and
write the FBoW vocabulary's structural facts (branching factor, descriptor
type/size, block layout offsets the port's block-walk needs) to
vocab_facts.txt.

Vocab facts are read directly from external/candidates/orb_vocab.fbow's own
header (8-byte magic signature 55824124, then a sizeof==120,
8-byte-aligned-double `Vocabulary::params` struct written raw via
`str.write((char*)&_params, sizeof(params))` in
external/candidates/stella_vslam/3rd/FBoW/src/fbow.cpp
Vocabulary::toStream/fromStream) -- this is the same fixed-layout read
sv_bow.c itself must do, so parsing it here (once, informationally) is a
faithful description of what the port needs, not a separate code path from
what sv_bow implements.

Requires runs/stella_port/reference_build/install (tools/build_stella_reference.py)
to already exist -- this tool links against that same install, like
stella_port/reference/driver does.

Usage:
    python3 tools/dump_stella_frame_bow.py fr1_xyz fr1_desk [--max-frames N]
"""
import argparse
import json
import os
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEPS = ROOT / "external/candidates/deps/root/usr"
BUILD_ROOT = ROOT / "runs/stella_port/reference_build"
INSTALL_DIR = BUILD_ROOT / "install"
TOOL_SRC = ROOT / "stella_port/reference_tools"
TOOL_BUILD = BUILD_ROOT / "frame_bow_tool_build"
BIN = TOOL_BUILD / "dump_frame_bow"
VOCAB = ROOT / "external/candidates/orb_vocab.fbow"
CFG = ROOT / "stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml"
DATA_ROOT = ROOT / "runs/orb_port/paper_bench/data"
OUT_ROOT = ROOT / "runs/stella_port/reference_frame_bow"

OPENCV_PKGCONFIG = "/tmp/pose-opencv/pkgconfig"

SEQ_DATA_DIRS = {
    "fr1_xyz": "rgbd_dataset_freiburg1_xyz",
    "fr1_desk": "rgbd_dataset_freiburg1_desk",
}

# fbow::Vocabulary::params field layout (3rd/FBoW/include/fbow/vocabulary.h):
#   char _desc_name_[50]; uint32_t _aligment,_nblocks; uint64_t
#   _desc_size_bytes_wp,_block_size_bytes_wp,_feature_off_start,
#   _child_off_start,_total_size; int32_t _desc_type,_desc_size;
#   uint32_t _m_k;
# Natural x86-64 struct padding -> desc_name[50] then 2 bytes pad, two
# uint32_t, 4 bytes pad, five uint64_t, two int32_t, one uint32_t, padded to
# a multiple of 8 -> sizeof == 120 (verified against the actual file: magic
# (8) + 120 + total_size == file size, and nblocks*block_size_bytes_wp ==
# total_size).
PARAMS_STRUCT = "<50sxx" + "II" + "xxxx" + "QQQQQ" + "iiI"


def parse_vocab_header(path: Path) -> dict:
    with open(path, "rb") as f:
        data = f.read(8 + 120)
    sig = struct.unpack_from("<Q", data, 0)[0]
    if sig != 55824124:
        raise ValueError(f"bad fbow signature: {sig}")
    fields = struct.unpack_from(PARAMS_STRUCT, data, 8)
    (desc_name, aligment, nblocks, desc_size_bytes_wp, block_size_bytes_wp,
     feature_off_start, child_off_start, total_size, desc_type, desc_size, m_k) = fields
    desc_name = desc_name.split(b"\x00", 1)[0].decode("ascii", "replace")
    file_size = path.stat().st_size
    # depth estimate: nblocks ~= (k^L - k) / (k - 1) for a tree with L
    # internal levels below the root (root's own children are block 0);
    # solve L from nblocks*(k-1)+k == k^L.
    levels_est = None
    if m_k > 1:
        import math
        val = nblocks * (m_k - 1) + m_k
        levels_est = round(math.log(val, m_k))
    return {
        "desc_name": desc_name,
        "aligment": aligment,
        "nblocks": nblocks,
        "desc_size_bytes_wp": desc_size_bytes_wp,
        "block_size_bytes_wp": block_size_bytes_wp,
        "feature_off_start": feature_off_start,
        "child_off_start": child_off_start,
        "total_size": total_size,
        "desc_type": desc_type,
        "desc_type_name": "CV_8UC1" if desc_type == 0 else str(desc_type),
        "desc_size": desc_size,
        "branching_factor_k": m_k,
        "levels_estimate": levels_est,
        "file_size": file_size,
        "header_size": 8 + 120,
        "computed_total_size_check": nblocks * block_size_bytes_wp,
        "weighting_scoring": (
            "FBoW stores a per-leaf float `weight` baked into the vocabulary "
            "file at training time (Vocabulary::block_node_info::weight); "
            "transform() just sums matched leaves' weights into the BoW map "
            "then L2-normalizes (Vocabulary::transform, fbow.cpp) -- no "
            "separate IDF/TF-IDF step at query time. Scoring between two "
            "BoWVectors is L2: score = 1 - sqrt(1 - sum(v_i*w_i)) "
            "(BoWVector::score, fbow.cpp), same formula sv_bow must use."
        ),
        "descriptor_matching": (
            "desc_type==CV_8UC1, desc_size==32 (ORB) -> Vocabulary::transform "
            "picks the L1_32bytes Computer on any x86-64 host (cpu::HW_x64 "
            "true), i.e. Hamming distance via 4x uint64 XOR + popcount "
            "(std::bitset<64>::count, exact integer result, no float/SIMD "
            "path needed for bit-exactness)."
        ),
        "bow_level": 4,
        "bow_level_note": (
            "stella_vslam/data/bow_vocabulary_util.cc compute_bow(): "
            "bow_vocab->transform(descriptors, 4, bow_vec, bow_feat_vec) -- "
            "level 4 is where BoWFeatVector node ids are captured during the "
            "same tree walk that produces bow_vec (Vocabulary::_transform2, "
            "vocabulary.h)."
        ),
    }


def pkgconfig(flag, pkg="opencv4"):
    env = os.environ.copy()
    env["PKG_CONFIG_PATH"] = OPENCV_PKGCONFIG
    out = subprocess.run(["pkg-config", flag, pkg], capture_output=True, text=True, env=env, check=True)
    return out.stdout.split()


def build_tool():
    env = os.environ.copy()
    env["PKG_CONFIG_PATH"] = OPENCV_PKGCONFIG + ":" + env.get("PKG_CONFIG_PATH", "")
    TOOL_BUILD.mkdir(parents=True, exist_ok=True)
    cxx_flags = "-O2 -DNDEBUG -ffp-contract=off -fno-fast-math"
    libarch = DEPS / "lib/x86_64-linux-gnu"
    localopencv_lib = "/tmp/localopencv/root/usr/lib/x86_64-linux-gnu"
    linker_flags = (f"-L{libarch} -L{localopencv_lib} -Wl,--allow-shlib-undefined "
                     f"-Wl,-rpath,{libarch} -Wl,-rpath,{INSTALL_DIR}/lib -Wl,-rpath,{localopencv_lib}")
    subprocess.run([
        "cmake", "-S", str(TOOL_SRC), "-B", str(TOOL_BUILD),
        f"-DSTELLA_REFERENCE_INSTALL_DIR={INSTALL_DIR}",
        f"-DCMAKE_PREFIX_PATH={INSTALL_DIR};{DEPS};{DEPS}/lib/cmake",
        f"-DCMAKE_CXX_FLAGS={cxx_flags}",
        f"-DCMAKE_EXE_LINKER_FLAGS={linker_flags}",
    ], check=True, env=env)
    subprocess.run(["cmake", "--build", str(TOOL_BUILD), "-j", str(os.cpu_count())], check=True, env=env)


def run_one(seq: str, max_frames: int):
    data_dir = DATA_ROOT / SEQ_DATA_DIRS[seq]
    if not data_dir.exists():
        print(f"skip {seq}: {data_dir} not found", file=sys.stderr)
        return
    out_dir = OUT_ROOT / seq
    out_dir.mkdir(parents=True, exist_ok=True)

    libarch = DEPS / "lib/x86_64-linux-gnu"
    env_path = (f"{INSTALL_DIR}/lib:{DEPS}/lib:{libarch}:"
                f"/tmp/pose-opencv/root/usr/lib/x86_64-linux-gnu:"
                f"/tmp/localopencv/root/usr/lib/x86_64-linux-gnu:/tmp/localopencv/root/usr/lib")
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = env_path
    env["OMP_NUM_THREADS"] = "1"

    cmd = [str(BIN), str(VOCAB), str(CFG), str(data_dir), str(out_dir), str(max_frames)]
    print("+", " ".join(cmd))
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    (out_dir / "stderr.log").write_text(proc.stdout + proc.stderr)
    if proc.returncode != 0:
        print(proc.stdout[-4000:], file=sys.stderr)
        print(proc.stderr[-4000:], file=sys.stderr)
        raise SystemExit(f"{seq}: dump_frame_bow failed (exit {proc.returncode})")
    print(f"{seq}: -> {out_dir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqs", nargs="+", choices=list(SEQ_DATA_DIRS.keys()))
    ap.add_argument("--max-frames", type=int, default=-1)
    ap.add_argument("--skip-build", action="store_true")
    args = ap.parse_args()

    if not INSTALL_DIR.exists():
        print(f"error: {INSTALL_DIR} not found -- run tools/build_stella_reference.py first", file=sys.stderr)
        return 1

    if not args.skip_build:
        build_tool()

    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    facts = parse_vocab_header(VOCAB)
    (OUT_ROOT / "vocab_facts.json").write_text(json.dumps(facts, indent=2))
    with open(OUT_ROOT / "vocab_facts.txt", "w") as f:
        for k, v in facts.items():
            f.write(f"{k}: {v}\n")
    print(f"vocab facts -> {OUT_ROOT / 'vocab_facts.txt'}")

    for seq in args.seqs:
        run_one(seq, args.max_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
