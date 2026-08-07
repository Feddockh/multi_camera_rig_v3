#!/usr/bin/env python3
"""Download model files from Google Drive into their destination directories.

Usage:
  python3 scripts/download_models.py --list
  python3 scripts/download_models.py --all
  python3 scripts/download_models.py foundation_stereo_checkpoint [other_key ...]

Models are defined in scripts/models_manifest.py.
"""
import argparse
import sys
from pathlib import Path

from models_manifest import MODELS

REPO_ROOT = Path(__file__).resolve().parent.parent


def require_gdown():
    try:
        import gdown  # noqa: F401
        return gdown
    except ImportError:
        print(
            "Missing dependency 'gdown'. Install it with:\n"
            "  pip3 install gdown\n"
            "or, if pip isn't installed:\n"
            "  sudo apt install python3-pip && pip3 install gdown",
            file=sys.stderr,
        )
        sys.exit(1)


def resolve_selection(keys, want_all):
    if want_all:
        selected = dict(MODELS)
    else:
        selected = {}
        for key in keys:
            if key not in MODELS:
                print(f"Unknown model '{key}'. Known models: {', '.join(MODELS)}", file=sys.stderr)
                sys.exit(1)
            selected[key] = MODELS[key]
    return selected


def total_size_mb(entry):
    sizes = [f.get("size_mb") for f in entry["files"]]
    return sum(sizes) if all(s is not None for s in sizes) else None


def print_list():
    for key, entry in MODELS.items():
        size = total_size_mb(entry)
        size_str = f"{size:.1f} MB" if size is not None else "size unknown"
        configured = "" if all(f.get("drive_id") for f in entry["files"]) else " [not configured]"
        print(f"  {key:28s} {entry['description']} ({size_str}){configured}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("keys", nargs="*", help="model keys to download (see --list)")
    parser.add_argument("--all", action="store_true", help="download every configured model")
    parser.add_argument("--list", action="store_true", help="list available models and exit")
    parser.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    parser.add_argument("--force", action="store_true", help="overwrite existing files without prompting")
    args = parser.parse_args()

    if args.list:
        print_list()
        return

    if not args.all and not args.keys:
        parser.print_help()
        sys.exit(1)

    selected = resolve_selection(args.keys, args.all)

    downloads = []  # (label, dest, drive_id, size_mb)
    for key, entry in selected.items():
        dest_dir = REPO_ROOT / entry["dest_dir"]
        for f in entry["files"]:
            if not f.get("drive_id"):
                print(f"Skipping '{key}/{f['filename']}': no drive_id configured in models_manifest.py")
                continue
            downloads.append((f"{key}/{f['filename']}", dest_dir / f["filename"], f["drive_id"], f.get("size_mb")))

    if not downloads:
        print("Nothing to download.")
        return

    print("The following files will be downloaded:")
    total_mb = 0
    for label, dest, _drive_id, size_mb in downloads:
        if size_mb is not None:
            total_mb += size_mb
        size_str = f"~{size_mb} MB" if size_mb is not None else "size unknown"
        print(f"  - {label}: {dest} ({size_str})")
    print(f"Total: ~{total_mb:.1f} MB")

    if not args.yes:
        reply = input("Proceed with download? [y/N] ").strip().lower()
        if reply not in ("y", "yes"):
            print("Aborted.")
            return

    gdown = require_gdown()

    for label, dest, drive_id, _size_mb in downloads:
        if dest.exists() and not args.force:
            reply = input(f"{dest} already exists. Overwrite? [y/N] ").strip().lower()
            if reply not in ("y", "yes"):
                print(f"Skipping '{label}'.")
                continue
        dest.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading '{label}' -> {dest}")
        gdown.download(id=drive_id, output=str(dest), quiet=False)


if __name__ == "__main__":
    main()
