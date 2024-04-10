# `multiple_object_tracking::cluster_detections`

Defined in `<multiple_object_tracking/clustering.hpp>`

```cpp
template <typename Detection>
[[nodiscard]]
auto cluster_detections(
    std::vector<Detection> detections,
    double distance_threshold
) -> std::vector<Cluster<Detection>>;
```

Group the `detections` into clusters based on their relative Euclidean distances. All detections whose Euclidean
distances are within `distance_threshold` will be grouped into the same cluster. Detections that could belong to
multiple clusters will non-deterministaclly be assigned to a single one.

## Template parameters

- `Detection` - the type of detections being clustered

## Parameters

- `detections` - the detections to be grouped into clusters
- `distance_threshold` - the Euclidean distance below which two detections will be considered part of the same cluster

## Return value

The clusters generated from the provided `detections`.
