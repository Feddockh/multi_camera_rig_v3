"""Manifest of downloadable model files (Google Drive) used by download_models.py.

To add or swap a model, add/edit an entry below. Each entry needs:
  description - human-readable label
  dest_dir    - destination directory, relative to the repo root
  files       - list of {drive_id, filename, size_mb} to download into dest_dir
                drive_id is the Google Drive file id (from the share link .../d/<ID>/view),
                and must point directly at the file, not a folder
                size_mb is approximate, shown in the confirmation prompt
"""

MODELS = {
    "foundation_stereo_checkpoint": {
        "description": "FoundationStereo ViT-Small checkpoint (11-33-40)",
        "dest_dir": "external/FoundationStereo/pretrained_models/11-33-40",
        "files": [
            {"drive_id": "1kVuORLuJI0aqz6u5ZrbyE960IBLTw3P9", "filename": "cfg.yaml", "size_mb": 0.001},
            {"drive_id": "1Ei-EBaF3EQA977zdjbXdmoXE7WuyJ1ib", "filename": "model_best_bp2.pth", "size_mb": 751.3},
        ],
    },
    "yolo_seg_lab": {
        "description": "YOLO-seg, lab-finetuned",
        "dest_dir": "multi_camera_rig_detection/models",
        "files": [
            {"drive_id": "1ytsGnicqiLmZX3gOFnMceOmFktvGsoDs", "filename": "best_lab_seg_v2.onnx", "size_mb": 107.3},
        ],
    },
}
