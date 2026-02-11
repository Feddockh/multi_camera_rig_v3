#!/usr/bin/env python3
"""
Standalone ArUco detection test script for debugging.
Includes robust parameter tuning for simulated/processed images.
"""
import cv2
import numpy as np
import sys


def create_detector_parameters():
    """Create detector parameters with proper API compatibility."""
    # Handle different OpenCV versions
    if hasattr(cv2.aruco, 'DetectorParameters'):
        # OpenCV 4.7+
        parameters = cv2.aruco.DetectorParameters()
    else:
        # Older OpenCV versions
        parameters = cv2.aruco.DetectorParameters_create()
    return parameters


def configure_parameters(parameters):
    """
    Configure parameters for robust detection in simulated/processed images.
    These settings are more tolerant of lighting variations and image artifacts.
    """
    # Adaptive thresholding - more lenient for varying lighting (like flash effect)
    parameters.adaptiveThreshWinSizeMin = 3 # (default: 3)
    parameters.adaptiveThreshWinSizeMax = 23 # (default: 23)
    parameters.adaptiveThreshWinSizeStep = 10 # (default: 10)
    parameters.adaptiveThreshConstant = 7 # (default: 7)
    
    # Contour filtering - allow smaller and larger markers
    parameters.minMarkerPerimeterRate = 0.03  # Allow smaller markers (default: 0.03)
    parameters.maxMarkerPerimeterRate = 4.0   # Allow larger markers (default: 4.0)
    parameters.polygonalApproxAccuracyRate = 0.03  # More tolerant polygon approximation (default: 0.03)
    
    # Corner detection - more lenient
    parameters.minCornerDistanceRate = 0.01  # Reduced from 0.05
    parameters.minMarkerDistanceRate = 0.01  # Reduced from 0.05
    
    # Edge/bit detection - important for 1-bit border markers
    parameters.minDistanceToBorder = 1  # Allow markers near image edge
    parameters.markerBorderBits = 1     # Match the border_bits=1 used in generation!
    
    # Perspective removal - helps with angled views
    parameters.perspectiveRemovePixelPerCell = 8
    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
    
    # Bit extraction - more tolerant thresholds
    parameters.maxErroneousBitsInBorderRate = 0.5  # Allow more border errors (default: 0.35)
    parameters.errorCorrectionRate = 0.8  # Use more error correction (default: 0.6)
    
    # Corner refinement for better accuracy
    if hasattr(parameters, 'cornerRefinementMethod'):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 30
        parameters.cornerRefinementMinAccuracy = 0.1
    
    return parameters

def detect_markers(img, aruco_dict, parameters):
    """Detect markers with proper API compatibility."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        # OpenCV 4.7+
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(img)
    else:
        # Older OpenCV versions
        corners, ids, rejected = cv2.aruco.detectMarkers(
            img, aruco_dict, parameters=parameters
        )
    return corners, ids, rejected

def test_detection(image_path, dictionary_name="DICT_4X4_50"):
    """Test ArUco detection on a single image with multiple parameter sets."""
    
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: Could not load image from {image_path}")
        return
    
    print(f"Image loaded: shape={img.shape}, dtype={img.dtype}")
    
    # Setup ArUco dictionary
    dict_mapping = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    }
    
    if dictionary_name not in dict_mapping:
        print(f"ERROR: Unknown dictionary '{dictionary_name}'")
        print(f"Available: {list(dict_mapping.keys())}")
        return
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_mapping[dictionary_name])
    
    # Preprocess image as grayscale for detection (Aruco works best in single channel)
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    
    print(f"\n{'='*60}")
    print(f"=== Test 1: DEFAULT parameters on grayscale ===")
    print(f"Dictionary: {dictionary_name}")
    
    parameters = create_detector_parameters()
    parameters = configure_parameters(parameters)
    corners, ids, rejected = detect_markers(gray, aruco_dict, parameters)
    
    num_detected = len(ids) if ids is not None else 0
    num_rejected = len(rejected) if rejected is not None else 0
    print(f"Detected: {num_detected} markers, Rejected: {num_rejected} candidates")
    
    if ids is not None:
        print(f"  IDs found: {ids.flatten().tolist()}")
    
    # Print out and save the results for this test
    print(f"\n{'='*60}")
    print(f"=== RESULTS ===")
    num_ids = len(ids) if ids is not None else 0
    print(f"Detected {num_ids} markers")
    
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            print(f"  Marker ID: {marker_id}")
            corner = corners[i].reshape(-1, 2)
            print(f"    Corners: {corner.astype(int).tolist()}")
    
    # Draw results - show best detection
    output_img = img.copy()
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(output_img, corners, ids)
    
    # Draw rejected in red (thin lines)
    if rejected is not None and len(rejected) > 0:
        for rejected_contour in rejected:
            pts = rejected_contour.reshape(-1, 2).astype(np.int32)
            cv2.polylines(output_img, [pts], True, (0, 0, 255), 1)
    
    output_path = image_path.rsplit('.', 1)[0] + f'_detected_{dictionary_name}.jpg'
    cv2.imwrite(output_path, output_img)
    print(f"\nSaved result to: {output_path}")
    
    # Print parameter recommendations
    print(f"\n{'='*60}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_aruco_detection.py <image_path> [dictionary]")
        print("Example: python test_aruco_detection.py test_image.jpg DICT_4X4_50")
        sys.exit(1)
    
    image_path = sys.argv[1]
    dictionary = sys.argv[2] if len(sys.argv) > 2 else "DICT_4X4_50"
    
    test_detection(image_path, dictionary)
