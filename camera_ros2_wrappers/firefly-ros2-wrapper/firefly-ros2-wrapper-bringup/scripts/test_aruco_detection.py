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


def configure_robust_parameters(parameters):
    """
    Configure parameters for robust detection in simulated/processed images.
    These settings are more tolerant of lighting variations and image artifacts.
    """
    # # Adaptive thresholding - more lenient for varying lighting (like flash effect)
    # parameters.adaptiveThreshWinSizeMin = 3
    # parameters.adaptiveThreshWinSizeMax = 23
    # parameters.adaptiveThreshWinSizeStep = 10
    # parameters.adaptiveThreshConstant = 7
    
    # # Contour filtering - allow smaller and larger markers
    # parameters.minMarkerPerimeterRate = 0.01  # Allow smaller markers (default: 0.03)
    # parameters.maxMarkerPerimeterRate = 4.0   # Allow larger markers (default: 4.0)
    # parameters.polygonalApproxAccuracyRate = 0.05  # More tolerant polygon approximation
    
    # # Corner detection - more lenient
    # parameters.minCornerDistanceRate = 0.01  # Reduced from 0.05
    # parameters.minMarkerDistanceRate = 0.01  # Reduced from 0.05
    
    # # Edge/bit detection - important for 1-bit border markers
    # parameters.minDistanceToBorder = 1  # Allow markers near image edge
    # parameters.markerBorderBits = 1     # Match the border_bits=1 used in generation!
    
    # # Perspective removal - helps with angled views
    # parameters.perspectiveRemovePixelPerCell = 8
    # parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
    
    # # Bit extraction - more tolerant thresholds
    # parameters.maxErroneousBitsInBorderRate = 0.5  # Allow more border errors (default: 0.35)
    # parameters.errorCorrectionRate = 0.8  # Use more error correction (default: 0.6)
    
    # # Corner refinement for better accuracy
    # if hasattr(parameters, 'cornerRefinementMethod'):
    #     parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    #     parameters.cornerRefinementWinSize = 5
    #     parameters.cornerRefinementMaxIterations = 30
    #     parameters.cornerRefinementMinAccuracy = 0.1
    
    return parameters


def preprocess_image(img):
    """
    Preprocess image to improve marker detection.
    Handles flash effects and lighting variations.
    """
    # Convert to grayscale
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    
    # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    # This helps normalize lighting from flash effects
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    
    return gray, enhanced


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
    
    # Preprocess image
    gray, enhanced = preprocess_image(img)
    
    # ========== Test 1: Default parameters on original grayscale ==========
    print(f"\n{'='*60}")
    print(f"=== Test 1: DEFAULT parameters on grayscale ===")
    print(f"Dictionary: {dictionary_name}")
    
    parameters = create_detector_parameters()
    corners, ids, rejected = detect_markers(gray, aruco_dict, parameters)
    
    num_detected = len(ids) if ids is not None else 0
    num_rejected = len(rejected) if rejected is not None else 0
    print(f"Detected: {num_detected} markers, Rejected: {num_rejected} candidates")
    
    if ids is not None:
        print(f"  IDs found: {ids.flatten().tolist()}")
    
    # ========== Test 2: Robust parameters on original grayscale ==========
    print(f"\n{'='*60}")
    print(f"=== Test 2: ROBUST parameters on grayscale ===")
    
    parameters = create_detector_parameters()
    parameters = configure_robust_parameters(parameters)
    corners2, ids2, rejected2 = detect_markers(gray, aruco_dict, parameters)
    
    num_detected2 = len(ids2) if ids2 is not None else 0
    num_rejected2 = len(rejected2) if rejected2 is not None else 0
    print(f"Detected: {num_detected2} markers, Rejected: {num_rejected2} candidates")
    
    if ids2 is not None:
        print(f"  IDs found: {ids2.flatten().tolist()}")
    
    # ========== Test 3: Robust parameters on CLAHE enhanced image ==========
    print(f"\n{'='*60}")
    print(f"=== Test 3: ROBUST parameters on CLAHE-enhanced image ===")
    
    corners3, ids3, rejected3 = detect_markers(enhanced, aruco_dict, parameters)
    
    num_detected3 = len(ids3) if ids3 is not None else 0
    num_rejected3 = len(rejected3) if rejected3 is not None else 0
    print(f"Detected: {num_detected3} markers, Rejected: {num_rejected3} candidates")
    
    if ids3 is not None:
        print(f"  IDs found: {ids3.flatten().tolist()}")
    
    # Use the best result
    results = [
        (corners, ids, rejected, "default_gray"),
        (corners2, ids2, rejected2, "robust_gray"),
        (corners3, ids3, rejected3, "robust_clahe"),
    ]
    
    # Pick the result with the most detections
    best_result = max(results, key=lambda x: len(x[1]) if x[1] is not None else 0)
    best_corners, best_ids, best_rejected, best_name = best_result
    
    print(f"\n{'='*60}")
    print(f"=== BEST RESULT: {best_name} ===")
    num_best = len(best_ids) if best_ids is not None else 0
    print(f"Detected {num_best} markers")
    
    if best_ids is not None:
        for i, marker_id in enumerate(best_ids.flatten()):
            print(f"  Marker ID: {marker_id}")
            corner = best_corners[i].reshape(-1, 2)
            print(f"    Corners: {corner.astype(int).tolist()}")
    
    # Draw results - show best detection
    output_img = img.copy()
    if best_ids is not None and len(best_ids) > 0:
        cv2.aruco.drawDetectedMarkers(output_img, best_corners, best_ids)
    
    # Draw rejected in red (thin lines)
    if best_rejected is not None and len(best_rejected) > 0:
        for rejected_contour in best_rejected:
            pts = rejected_contour.reshape(-1, 2).astype(np.int32)
            cv2.polylines(output_img, [pts], True, (0, 0, 255), 1)
    
    output_path = image_path.rsplit('.', 1)[0] + f'_detected_{best_name}.jpg'
    cv2.imwrite(output_path, output_img)
    print(f"\nSaved result to: {output_path}")
    
    # Also save the enhanced image for debugging
    enhanced_path = image_path.rsplit('.', 1)[0] + '_clahe_enhanced.jpg'
    cv2.imwrite(enhanced_path, enhanced)
    print(f"Saved CLAHE enhanced image to: {enhanced_path}")
    
    # Print parameter recommendations
    print(f"\n{'='*60}")
    print("=== PARAMETER RECOMMENDATIONS ===")
    print("If detection still fails, check:")
    print("  1. Marker border_bits: Generator uses 1, detector expects same")
    print("  2. Dictionary match: Ensure same dict for generation and detection")
    print("  3. Image quality: Check the CLAHE-enhanced image for marker visibility")
    print("  4. Marker size: Very small markers need minMarkerPerimeterRate < 0.01")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_aruco_detection.py <image_path> [dictionary]")
        print("Example: python test_aruco_detection.py test_image.jpg DICT_4X4_50")
        sys.exit(1)
    
    image_path = sys.argv[1]
    dictionary = sys.argv[2] if len(sys.argv) > 2 else "DICT_4X4_50"
    
    test_detection(image_path, dictionary)
