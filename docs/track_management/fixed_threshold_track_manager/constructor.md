# `multiple_object_tracking::FixedThresholdTrackManager<Track>::FixedThresholdTrackManager`

```cpp
explicit FixedThresholdTrackManager(
    multiple_object_tracking::PromotionThreshold p,
    multiple_object_tracking::RemovalThreshold r
);
```

Constructs a `multiple_object_tracking::FixedThresholdTrackManager`.

## Parameters

- `p` - occurrence threshold at or above which a detection will be promoted to _confirmed_
        NOTE: New detection for a confirmed track will force the occurrence to be at the threshold
- `r` - occurrence threshold at or below which a detection will be removed from management
