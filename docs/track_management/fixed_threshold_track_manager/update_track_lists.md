# `multiple_object_tracking::FixedThresholdTrackManager<Track>::update_track_lists`

```cpp
auto update_track_lists(const AssociationMap & associations) -> void;
```

Updates each track's status based on the associations. Each track will either be promoted, demoted, removed, or remain unchanged.

## Parameters

- `associations` - a map of track-to-detection associations indicating the detections associated with each track

## Return value

(none)
