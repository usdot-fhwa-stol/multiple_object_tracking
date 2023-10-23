# `multiple_object_tracking::Cluster`

Defined in header `<multpile_object_tracking/clustering.hpp>`

```cpp
template <
    typename Detection
> class Cluster;
```

This class contains a sequence of detections that are within a specific Euclidean distance of each other.

## Member functions

|                                       |                          |
| ------------------------------------- | ------------------------ |
| (constructor) _(implicitly declared)_ | constructs the `Cluster` |
| (destructor) _(implicitly declared)_  | destructs the `Cluster`  |

### Observers

|                                    |                                                             |
| ---------------------------------- | ----------------------------------------------------------- |
| [`get_centroid`][get_centroid_doc] | returns a point representing the cluster's geometric center |

[get_centroid_doc]: get_centroid.md

### Accessors

|                                        |                                                    |
| -------------------------------------- | -------------------------------------------------- |
| [`get_detections`][get_detections_doc] | access the detections contained within the cluster |

[get_detections_doc]: get_detections.md

### Capacity

|                            |                                     |
| -------------------------- | ----------------------------------- |
| [`is_empty`][is_empty_doc] | checks whether the cluster is empty |

[is_empty_doc]: is_empty.md

### Modifiers

|                                            |                                      |
| ------------------------------------------ | ------------------------------------ |
| [`clear`][clear_doc]                       | clears the cluster's contents        |
| [`add_detection`][add_detection_doc]       | adds a detection to the cluster      |
| [`remove_detection`][remove_detection_doc] | removes a detection from the cluster |

[clear_doc]: clear.md
[add_detection_doc]: add_detection.md
[remove_detection_doc]: remove_detection.md
