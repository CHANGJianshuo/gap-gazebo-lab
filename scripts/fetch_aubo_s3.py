#!/usr/bin/env python3
"""Fetch the official S3 model subset at a fixed upstream commit, verifying blobs."""

import hashlib
import json
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest_path = ROOT / "third_party/aubo_description/source_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    destination = manifest_path.parent / "upstream"
    for entry in manifest["files"]:
        relative = Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise SystemExit(f"Invalid upstream path: {relative}")
        path = destination / relative
        if path.is_file():
            content = path.read_bytes()
            blob = hashlib.sha1(b"blob " + str(len(content)).encode() + b"\0" + content).hexdigest()
            if blob == entry["git_blob_sha1"]:
                print(f"Verified existing {relative}", flush=True)
                continue
            raise SystemExit(f"Existing file differs from upstream: {path}; refusing to overwrite it.")
        url = f"https://raw.githubusercontent.com/AuboRobot/aubo_description/{manifest['commit']}/{relative.as_posix()}"
        request = urllib.request.Request(url, headers={"User-Agent": "meituan-challenge-model-fetch"})
        with urllib.request.urlopen(request, timeout=30) as response:
            content = response.read()
        blob = hashlib.sha1(b"blob " + str(len(content)).encode() + b"\0" + content).hexdigest()
        if blob != entry["git_blob_sha1"] or len(content) != entry["size_bytes"]:
            raise SystemExit(f"Upstream checksum/size mismatch: {relative}")
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".download")
        temporary.write_bytes(content)
        temporary.replace(path)
        print(f"Fetched {relative} ({len(content)} bytes)", flush=True)
    print(f"Verified {len(manifest['files'])} official files at {manifest['commit']}", flush=True)


if __name__ == "__main__":
    main()
