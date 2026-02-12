# Semantic Occupancy Mapping Integration Plan

## Overview

This document outlines the integration plan for semantic pointclouds with the occupancy mapping subpackage. The semantic pointcloud node now publishes `sensor_msgs/PointCloud2` messages with custom fields that include semantic class and confidence information alongside traditional XYZ and RGB data.

## Custom Point Type Structure

The semantic pointcloud uses the following point structure:

```cpp
struct PointXYZRGBSemanticConfidence {
    float x;            // 3D position X
    float y;            // 3D position Y
    float z;            // 3D position Z
    float rgb;          // Packed RGB color (PCL/RViz compatible)
    int32_t class_id;   // Semantic class identifier
    float confidence;   // Detection confidence [0.0, 1.0]
}
```

### PointCloud2 Field Layout

The published `sensor_msgs/PointCloud2` message contains 6 fields:

| Field Name | Offset | Datatype | Size | Description |
|------------|--------|----------|------|-------------|
| x | 0 | FLOAT32 | 4 bytes | X coordinate |
| y | 4 | FLOAT32 | 4 bytes | Y coordinate |
| z | 8 | FLOAT32 | 4 bytes | Z coordinate |
| rgb | 12 | FLOAT32 | 4 bytes | Packed RGB (compatible with standard viewers) |
| class_id | 16 | INT32 | 4 bytes | Semantic class (-1 for background) |
| confidence | 20 | FLOAT32 | 4 bytes | Confidence score |

Total point size: **24 bytes per point**

## Occupancy Mapping Integration

### Recommended Architecture

#### 1. Semantic OctoMap Extension

Create a new package `semantic_octomap` that extends the base OctoMap functionality:

```
semantic_octomap/
├── include/
│   └── semantic_octomap/
│       ├── semantic_octree.hpp          # Extended OcTree class
│       ├── semantic_octree_node.hpp     # Node with semantic data
│       └── semantic_octomap_server.hpp  # ROS2 wrapper
├── src/
│   ├── semantic_octree.cpp
│   ├── semantic_octree_node.cpp
│   └── semantic_octomap_server_node.cpp
└── CMakeLists.txt
```

#### 2. Semantic OcTree Node Structure

Extend `OcTreeNode` to store semantic information:

```cpp
class SemanticOcTreeNode : public OcTreeNode {
public:
    // Inherited: occupancy probability
    
    // Semantic information
    int32_t semantic_class_;      // Most likely class
    float semantic_confidence_;    // Confidence in classification
    
    // Optional: Multi-hypothesis tracking
    std::map<int32_t, float> class_distribution_;  // class_id -> confidence
    
    // Update methods
    void updateSemantics(int32_t class_id, float confidence);
    int32_t getSemanticClass() const;
    float getSemanticConfidence() const;
};
```

### 3. Integration Strategy

#### Point Cloud Processing Pipeline

```
Semantic PointCloud2
        ↓
   Parse custom fields
        ↓
   Group by voxel
        ↓
   Max-fusion update per voxel
        ↓
   Update occupancy + semantics
```

#### Voxel-Level Fusion

For each voxel, track observations and apply max-fusion logic:

**Same-Class Fusion:**
```python
# When new observation matches existing class
updated_confidence = (prev_confidence + new_confidence) / 2.0 + confidence_boost
updated_confidence = clip(updated_confidence, 0.0, 1.0)
```

**Different-Class Fusion:**
```python
# When new observation differs from existing class
if new_confidence > prev_confidence:
    chosen_class = new_class
    chosen_confidence = new_confidence * (1.0 - mismatch_penalty)
else:
    chosen_class = prev_class
    chosen_confidence = prev_confidence * (1.0 - mismatch_penalty)
```

### 4. Implementation Steps

#### Phase 1: Basic Semantic Integration
1. Create `SemanticOcTreeNode` class extending `OcTreeNode`
2. Add semantic fields (class_id, confidence) to node structure
3. Implement basic semantic update logic (max-fusion)
4. Create ROS2 service interface for querying semantic information

#### Phase 2: PointCloud2 Parser
1. Create utility to parse custom PointCloud2 fields:
   ```cpp
   struct SemanticPoint {
       Eigen::Vector3d position;
       uint8_t r, g, b;
       int32_t class_id;
       float confidence;
   };
   
   std::vector<SemanticPoint> parseSemanticPointCloud(
       const sensor_msgs::msg::PointCloud2& cloud);
   ```

2. Implement voxel-level grouping:
   ```cpp
   std::map<octomap::OcTreeKey, std::vector<SemanticPoint>> 
       groupByVoxel(const std::vector<SemanticPoint>& points);
   ```

3. Per-voxel observation selection (highest confidence):
   ```cpp
   SemanticObservation selectBestObservation(
       const std::vector<SemanticPoint>& voxel_points);
   ```

#### Phase 3: Server Node
1. Subscribe to `/stereo/points` (semantic pointcloud)
2. Subscribe to camera pose/odometry for sensor origin
3. Apply ray-casting for occupancy updates
4. Apply semantic fusion for classification updates
5. Publish:
   - Binary occupancy map
   - Semantic occupancy map (colored by class)
   - Uncertainty visualization (colored by confidence)

#### Phase 4: Visualization & Query
1. Create RViz plugin for semantic occupancy visualization
2. Implement service calls:
   - `GetSemanticClass(point) -> (class_id, confidence)`
   - `GetOccupiedVoxelsWithClass(class_id) -> PointCloud2`
   - `GetClassDistribution(bbox) -> map<class_id, count>`
3. Export semantic map to file format

### 5. Configuration Parameters

#### Semantic Fusion Parameters
```yaml
semantic_octomap_server:
  ros__parameters:
    # OctoMap parameters
    resolution: 0.05  # 5cm voxels
    max_range: 10.0
    
    # Semantic fusion parameters
    background_class_id: -1
    confidence_boost: 0.1          # Boost for same-class observations
    mismatch_penalty: 0.1          # Penalty for different-class observations
    min_confidence_threshold: 0.3  # Ignore low-confidence observations
    
    # Multi-hypothesis tracking (optional)
    track_class_distribution: false
    max_classes_per_voxel: 3
    
    # Update strategy
    fusion_mode: "max"  # max | average | bayesian
    decay_factor: 0.95  # Decay old observations over time
```

### 6. Expected Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│ Firefly Camera System                                        │
├─────────────────────────────────────────────────────────────┤
│ ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│ │ Left Camera  │  │ Right Camera │  │ Detection    │       │
│ │   (RGB)      │  │   (RGB)      │  │   Node       │       │
│ └──────┬───────┘  └──────┬───────┘  └──────┬───────┘       │
│        │                  │                  │               │
│        └──────────┬───────┘                  │               │
│                   ↓                          │               │
│        ┌──────────────────────┐              │               │
│        │ Foundation Stereo    │              │               │
│        │ Matcher (TensorRT)   │              │               │
│        └──────────┬───────────┘              │               │
│                   │                          │               │
│                   │ disparity                │ detections    │
│                   ↓                          ↓               │
│        ┌──────────────────────────────────────┐             │
│        │ Semantic PointCloud Node             │             │
│        │ - Normal mode: XYZ + RGB             │             │
│        │ - Semantic mode: XYZ + RGB + Class   │             │
│        └──────────┬───────────────────────────┘             │
└────────────────────┼──────────────────────────────────────────┘
                     │
                     │ /stereo/points (PointCloud2)
                     ↓
         ┌───────────────────────────┐
         │ Semantic OctoMap Server   │
         │ - Occupancy mapping       │
         │ - Semantic fusion         │
         │ - Query interface         │
         └───────────┬───────────────┘
                     │
                     ├─→ Binary OctoMap
                     ├─→ Semantic OctoMap  
                     ├─→ Class-filtered clouds
                     └─→ Confidence visualization
```

### 7. Testing Strategy

#### Unit Tests
1. Point cloud parsing (verify field extraction)
2. Voxel grouping (verify spatial binning)
3. Semantic fusion logic (verify confidence updates)
4. Class distribution tracking

#### Integration Tests
1. End-to-end pipeline with synthetic data
2. Benchmark performance (points/sec, latency)
3. Memory usage profiling
4. Accuracy vs. ground truth semantic maps

#### Real-World Tests
1. Static scene reconstruction with known classes
2. Dynamic scene updates (moving objects)
3. Multi-view consistency checks
4. Long-term mapping stability

### 8. Performance Considerations

- **Voxel-level grouping prevents over-counting**: Multiple points in same voxel → single update
- **Confidence-based selection**: Only highest-confidence observation per voxel processed
- **Lazy evaluation**: Defer tree updates until query or save
- **Spatial hashing**: Use OcTreeKey for fast voxel lookup
- **Pruning**: Periodically remove low-confidence or inconsistent voxels

### 9. Future Enhancements

1. **Temporal consistency**: Track class changes over time
2. **Multi-robot fusion**: Merge semantic maps from multiple agents
3. **Active learning**: Identify regions with low confidence for targeted sensing
4. **Semantic SLAM**: Use semantic landmarks for localization
5. **Class-specific mapping**: Different resolution/parameters per class

## References

- OctoMap: https://octomap.github.io/
- PCL PointCloud2: http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
- Semantic OctoMap Paper: https://arxiv.org/abs/1703.00748

## Contact & Maintenance

This integration plan should be reviewed and updated as the system evolves. Key considerations:
- Detection message format (TODO: finalize when vision_msgs or custom msgs are ready)
- Coordinate frame conventions
- Performance requirements for real-time operation
