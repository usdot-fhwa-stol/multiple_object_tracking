# `multiple_object_tracking::FixedThresholdTrackManager<Track>::set_removal_threshold_and_update`

```cpp
auto set_removal_threshold_and_update(const RemovalThreshold & threshold) noexcept -> void;
```

Sets the removal threshold to a new value and prunes tracks based on the new threshold.

## Parameters

- `threshold` - occurrence threshold at or below which a detection will be removed from management

## Return value

(none)
