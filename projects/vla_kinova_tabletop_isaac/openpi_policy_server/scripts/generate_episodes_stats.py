#!/usr/bin/env python3
"""Prepare a Lerobot Studio v2.1 export for use with openpi.

Fixes three things that Lerobot Studio gets wrong:

1. meta/info.json — fixes the data_path and video_path templates.
   Studio writes {chunk_index}/{file_index} but the actual files use
   episode_{episode_index:06d}, and the openpi lerobot loader passes
   {episode_chunk} and {episode_index} as format keys.

2. meta/episodes_stats.jsonl — generates the nested per-episode stats file
   that the openpi lerobot loader requires. Studio stores the same data in
   meta/episodes.jsonl using flat slash-separated keys; this script reformats
   them without recomputing anything.

3. data/**/*.parquet — fixes the HuggingFace schema metadata embedded in each
   parquet file. Studio (via an older lerobot/datasets) writes "_type": "List"
   for fixed-length vector features, but datasets >= 3.x only recognises
   "_type": "Sequence". This only patches the key-value metadata; the Arrow
   column data is not re-encoded.

Usage
-----
Run with any Python 3 interpreter. pyarrow must be available (it is in both
the lerobot_ros and openpi venvs):

    python generate_episodes_stats.py /path/to/dataset@v2.1

Then push the whole folder to HF using the lerobot_ros venv as usual:

    hf upload <repo-id> /path/to/dataset@v2.1 . \\
        --repo-type dataset --revision v2.1 \\
        --commit-message "Fix parquet schema, info.json templates, add episodes_stats.jsonl"

Future datasets
---------------
After every Lerobot Studio v2.1 export, run this script before pushing to HF.
"""

import argparse
import json
import pathlib


# Templates expected by the openpi lerobot loader (get_data_file_path /
# get_video_file_path in lerobot_dataset.py). Must match actual file names.
_CORRECT_DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
_CORRECT_VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"


def fix_info_json(root: pathlib.Path) -> None:
    info_path = root / "meta" / "info.json"
    with open(info_path) as f:
        info = json.load(f)

    changed = False
    if info.get("data_path") != _CORRECT_DATA_PATH:
        info["data_path"] = _CORRECT_DATA_PATH
        changed = True
    if info.get("video_path") != _CORRECT_VIDEO_PATH:
        info["video_path"] = _CORRECT_VIDEO_PATH
        changed = True

    if changed:
        with open(info_path, "w") as f:
            json.dump(info, f, indent=2)
        print("Fixed meta/info.json path templates")
    else:
        print("meta/info.json path templates already correct")


def _fix_feature_types(obj: object) -> bool:
    """Recursively replace '_type': 'List' with '_type': 'Sequence'. Returns True if changed."""
    changed = False
    if isinstance(obj, dict):
        if obj.get("_type") == "List":
            obj["_type"] = "Sequence"
            changed = True
        for v in obj.values():
            changed |= _fix_feature_types(v)
    elif isinstance(obj, list):
        for item in obj:
            changed |= _fix_feature_types(item)
    return changed


def fix_parquet_schemas(root: pathlib.Path) -> None:
    import pyarrow.parquet as pq

    parquets = sorted((root / "data").rglob("*.parquet"))
    if not parquets:
        print("No parquet files found under data/, skipping schema fix")
        return

    fixed = 0
    for path in parquets:
        table = pq.read_table(path)
        raw_meta = dict(table.schema.metadata or {})
        hf_bytes = raw_meta.get(b"huggingface")
        if not hf_bytes:
            continue
        hf_meta = json.loads(hf_bytes)
        if _fix_feature_types(hf_meta):
            raw_meta[b"huggingface"] = json.dumps(hf_meta).encode()
            pq.write_table(table.replace_schema_metadata(raw_meta), path)
            fixed += 1

    if fixed:
        print(f"Fixed parquet schema metadata in {fixed}/{len(parquets)} files (List → Sequence)")
    else:
        print(f"Parquet schema metadata already correct in all {len(parquets)} files")


def generate_episodes_stats(root: pathlib.Path) -> None:
    src = root / "meta" / "episodes.jsonl"
    dst = root / "meta" / "episodes_stats.jsonl"

    if not src.exists():
        raise FileNotFoundError(
            f"Source not found: {src}\n"
            "This script requires meta/episodes.jsonl, which is present in v2.1\n"
            "datasets exported by Lerobot Studio."
        )

    if dst.exists():
        print("meta/episodes_stats.jsonl already exists, nothing to do")
        return

    print("Generating meta/episodes_stats.jsonl ...")
    count = 0
    with open(src) as fin, open(dst, "w") as fout:
        for raw in fin:
            ep = json.loads(raw)
            ep_idx = ep.get("episode_index")
            if ep_idx is None:
                continue

            stats: dict = {}
            for key, value in ep.items():
                if not key.startswith("stats/"):
                    continue
                # key format: "stats/<feature_name>/<stat_name>"
                # feature names may contain "." but never "/"
                _, feature, stat = key.split("/", 2)
                stats.setdefault(feature, {})[stat] = value

            fout.write(json.dumps({"episode_index": ep_idx, "stats": stats}) + "\n")
            count += 1

    print(f"Done. Written {count} episodes to {dst}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", help="Local v2.1 dataset root directory")
    args = parser.parse_args()

    root = pathlib.Path(args.root).expanduser().resolve()
    fix_info_json(root)
    fix_parquet_schemas(root)
    generate_episodes_stats(root)


if __name__ == "__main__":
    main()
