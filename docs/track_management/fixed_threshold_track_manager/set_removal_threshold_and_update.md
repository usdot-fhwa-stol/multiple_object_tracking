# `multiple_object_tracking::FixedThresholdTrackManager<Track>::set_confirmed_to_removal_threshold_and_update`

```cpp
auto set_confirmed_to_removal_threshold_and_update(const RemovalThreshold & threshold) noexcept -> void;
```

Sets the removal threshold to a new value and prunes tracks based on the new threshold.

## Parameters

- `threshold` - nonoccurrence threshold at or more which a Track is consecutively missed to be tagged for removal
NOTE: Detections get removed at occurrence 0 by default.

## Return value

(none)
