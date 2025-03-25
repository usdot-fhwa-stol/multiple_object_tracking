# Multiple object tracking library

This library provides functions and types to help user-programmers build tracking and estimation pipelines.

## Getting started

- [Multiple object tracking overview](multiple_object_tracking_overview.md)
- API reference

## Related projects and resources

- [Stone-soup][stone_soup_github]: A software project to provide the target tracking community with a framework for
  the development and testing of tracking algorithms.
- [Understanding Sensor Fusion and Tracking][mathworks_series_url]: This video series provides an overview of sensor
  fusion and multi-object tracking in autonomous systems. Starting with sensor fusion to determine positioning and
  localization, the series builds up to tracking single objects with an IMM filter, and completes with the topic of
  multi-object tracking.

[stone_soup_github]: https://github.com/dstl/Stone-Soup
[mathworks_series_url]: https://www.mathworks.com/videos/series/understanding-sensor-fusion-and-tracking.html

## Dynamic objects

|                                                                 |                                                                             |
| --------------------------------------------------------------- | --------------------------------------------------------------------------- |
| [`multiple_object_tracking::DynamicObject`][dynamic_object_doc] | generic collection of data for an object that moves through its environment |
| [`multiple_object_tracking::Detection`][detection_doc]          | collection of data related to a detected object (_detection_)               |
| [`multiple_object_tracking::Track`][track_doc]                  | collection data related to a tracked object (_track_)                       |

[dynamic_object_doc]: dynamic_objects/dynamic_object.md
[detection_doc]: dynamic_objects/detection.md
[track_doc]: dynamic_objects/track.md

## Temporal alignment

Temporal alignment...

### Global objects

|                                                                                            |                                                                                                                     |
| ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| [`multiple_object_tracking::default_unscented_transform`][default_unscented_transform_doc] | a [`multiple_object_tracking::UnscentedTransform`][unscented_transform_doc] instance with pre-defined tuning values |

[default_unscented_transform_doc]: temporal_alignment/default_unscented_transform.md

### Propagators

|                                                                           |                                                                                     |
| ------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| [`multiple_object_tracking::UnscentedTransform`][unscented_transform_doc] | propagate a dynamic object using an [unscented transform][unscented_transform_link] |

[unscented_transform_doc]: temporal_alignment/unscented_transform.md
[unscented_transform_link]: https://en.wikipedia.org/wiki/Unscented_transform

### Modifying operations

|                                              |                                                                          |
| -------------------------------------------- | ------------------------------------------------------------------------ |
| [`propagate_to_time`][propagate_to_time_doc] | update a dynamic object's state (and related fields) to a specified time |

[propagate_to_time_doc]: temporal_alignment/propagate_to_time.md

### Non-modifying operations

|                                          |                                                                   |
| ---------------------------------------- | ----------------------------------------------------------------- |
| [`predict_to_time`][predict_to_time_doc] | copy a dynamic object and propagates the copy to a specified time |

[predict_to_time_doc]: temporal_alignment/predict_to_time.md

## Scoring

### Scoring metrics

|                                                                |                                                                                            |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| [`euclidean_distance_score`][euclidean_distance_score_doc]     | calculate a pair-wise score based on the [Euclidean distance][euclidean_distance_link]     |
| [`mahalanobis_distance_score`][mahalanobis_distance_score_doc] | calculate a pair-wise score based on the [Mahalanobis distance][mahalanobis_distance_link] |

[euclidean_distance_score_doc]: scoring/euclidean_distance_score.md
[mahalanobis_distance_score_doc]: scoring/mahalanobis_distance_score.md
[euclidean_distance_link]: https://en.wikipedia.org/wiki/Euclidean_distance
[mahalanobis_distance_link]: https://en.wikipedia.org/wiki/Mahalanobis_distance

### Non-modifying operations

|                                                                  |                                                                                      |
| ---------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| [`score_tracks_and_detections`][score_tracks_and_detections_doc] | calculate pair-wise scores for a collection of tracks and a collection of detections |

[score_tracks_and_detections_doc]: scoring/score_tracks_and_detections.md

## Gating

### Modifying operations

|                                                                                  |                                                             |
| -------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| [`prune_track_and_detection_scores_if`][prune_track_and_detection_scores_if_doc] | remove all track-to-detection scores satisfying a predicate |

[prune_track_and_detection_scores_if_doc]: gating/prune_track_and_detection_scores_if.md

## Association (detection-to-track matching)

### Type definitions

| Type                                       | Definition                          |
| ------------------------------------------ | ----------------------------------- |
| `multiple_object_tracking::AssociationMap` | `std::map<Uuid, std::vector<Uuid>>` |

### Global objects

|                           |     |
| ------------------------- | --- |
| `gnn_association_visitor` |     |

### Associators

|     |           |
| --- | --------- |
| ``  | Something |

### Predicates

#### [`multiple_object_tracking::HasAssociation`][has_association_doc]

This class is a function object that can be used as a unary predicate to check
if a specified object (track or detection) has an association within a given
association map.

[has_association_doc]: track_matching/has_association/main.md

|                                                   |     |
| ------------------------------------------------- | --- |
| `multiple_object_tracking::isAssociatedDetection` |     |
| `multiple_object_tracking::isAssociatedTrack`     |     |

## Non-modifying operations

|                                  |     |
| -------------------------------- | --- |
| `score_matrix_from_score_map`    |     |
| `cost_matrix_from_score_matrix`  |     |
| `get_element_at`                 |     |
| `association_map_from_score_map` |     |
| `gnn_associator`                 |     |
| ` print_association_map`         |     |
| `associate_detections_to_tracks` |     |

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

## Fusion

### Fusers

|     |           |
| --- | --------- |
| ``  | Something |
