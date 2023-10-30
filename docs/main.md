# Multiple object tracking library

## Association (detection-to-track matching)

### Predicates

#### [`multiple_object_tracking::HasAssociation`][has_association_doc]

This class is a function object that can be used as a unary predicate to check
if a specified object (track or detection) has an association within a given
association map.

[has_association_doc]: track_matching/has_association/main.md

## Clustering

### [`multiple_object_tracking::Cluster`][cluster_doc]

This class contains a sequence of detections that are within a specific Euclidean distance of each other.

[cluster_doc]: clustering/cluster/index.md

### [`multiple_object_tracking::Point`][point_doc]

This class represents the geometric center of a cluster. It is a sub-vector of `multiple_object_tracking::CtrvState` and `multiple_object_tracking::CtraState` containing the common vector elements.

[point_doc]: clustering/point/index.md

### Clustering operations

|                                                |                                                                                            |
| ---------------------------------------------- | ------------------------------------------------------------------------------------------ |
| [`cluster_detections`][cluster_detections_doc] | group a collection of detections into clusters based on their relative Euclidean distances |

[cluster_detections_doc]: clustering/cluster_detections.md

## Track maintenance

### [`multiple_object_tracking::FixedThresholdTrackManager`][fixed_threshold_track_manager_doc]

The templated class [`multiple_object_tracking::FixedThresholdTrackManager`][fixed_threshold_track_manager_doc]
stores a list of `multiple_object_tracking::Track`s and manages their statuses (_tentative_ or
_confirmed_) using a fixed-threshold policy.

[fixed_threshold_track_manager_doc]: track_management/fixed_threshold_track_manager/main.md
