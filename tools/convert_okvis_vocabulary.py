#!/usr/bin/env python3
"""Convert the OKVIS2 BRISK DBoW2 vocabulary (external/vio/okvis2/resources/small_voc.yml.gz, OpenCV FileStorage YAML written by
DBoW2) into the binary payload ok_dbow_voc_parse() reads (layout in okvis_port/c/ok_dbow.h; identical to record 170 of place.bin,
patch 0014, which dumps the tree as Frontend::DBoW loaded it). Usage:

    python3 tools/convert_okvis_vocabulary.py [in.yml.gz] [out.bin]        (default out: runs/okvis_port/vocabulary/small_voc.bin)
    python3 tools/convert_okvis_vocabulary.py --check place.bin            compare with the dump of the reference (bitwise)

The vocabulary is data of the OKVIS2 repository (provenance undocumented, docs/okvis2_license_audit.md); the converted file lives
under the gitignored runs/ tree and is never committed. DBoW2 loader semantics (TemplatedVocabulary::load): nodes are listed with
their parent, the children of a node are appended in listing order, words map a word id to a node id, descriptors are 48 decimal bytes."""
import gzip, re, struct, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEF_IN = ROOT / "external/vio/okvis2/resources/small_voc.yml.gz"
DEF_OUT = ROOT / "runs/okvis_port/vocabulary/small_voc.bin"


def convert(path):
    text = gzip.open(path, "rt").read()
    k = int(re.search(r"\bk:\s*(\d+)", text).group(1))
    L = int(re.search(r"\bL:\s*(\d+)", text).group(1))
    scoring = int(re.search(r"scoringType:\s*(\d+)", text).group(1))
    weighting = int(re.search(r"weightingType:\s*(\d+)", text).group(1))
    node_re = re.compile(r"nodeId:\s*(\d+),\s*parentId:\s*(\d+),\s*weight:\s*([-+0-9.eE]+),\s*descriptor:\"([^\"]*)\"")
    nodes = {0: dict(id=0, parent=0, word=0, weight=0.0, children=[], desc=b"")}
    for m in node_re.finditer(text):
        nid, pid = int(m.group(1)), int(m.group(2))
        nodes.setdefault(nid, dict(id=nid, parent=0, word=0, weight=0.0, children=[], desc=b""))
        nodes.setdefault(pid, dict(id=pid, parent=0, word=0, weight=0.0, children=[], desc=b""))
        nodes[nid].update(parent=pid, weight=float(m.group(3)), desc=bytes(int(x) for x in m.group(4).split()))
        nodes[pid]["children"].append(nid)
    words = 0
    for m in re.finditer(r"wordId:\s*(\d+),\s*nodeId:\s*(\d+)", text):
        nodes[int(m.group(2))]["word"] = int(m.group(1)); words += 1
    n = max(nodes) + 1
    out = bytearray(struct.pack("<6I", k, L, weighting, scoring, n, words))
    for i in range(n):
        nd = nodes[i]
        out += struct.pack("<3I", nd["id"], nd["parent"], nd["word"]) + struct.pack("<d", nd["weight"])
        out += struct.pack("<I", len(nd["children"])) + b"".join(struct.pack("<I", c) for c in nd["children"])
        out += struct.pack("<I", len(nd["desc"])) + nd["desc"]
    return bytes(out)


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--check":
        data = Path(sys.argv[2]).read_bytes()
        o = 0
        while o < len(data):
            tag, ln = struct.unpack_from("<IQ", data, o)
            if tag == 170:
                ref = data[o + 12:o + 12 + ln]
                mine = convert(DEF_IN)
                print("vocabulary payload", len(mine), "bytes;", "IDENTICAL to the reference dump" if mine == ref else "DIFFERENT from the reference dump")
                return 0 if mine == ref else 1
            o += 12 + ln
        print("no record 170 in", sys.argv[2]); return 2
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else DEF_IN
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else DEF_OUT
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(convert(src))
    print("wrote", dst, dst.stat().st_size, "bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
