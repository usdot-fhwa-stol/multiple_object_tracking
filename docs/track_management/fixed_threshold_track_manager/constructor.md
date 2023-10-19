# `multiple_object_tracking::FixedThresholdTrackManager<Track>::FixedThresholdTrackManager`

```cpp
explicit FixedThresholdTrackManager(
    multiple_object_tracking::PromotionThreshold p,
    multiple_object_tracking::RemovalThreshold r
);
```
Constructs a `multiple_object_tracking::FixedThresholdTrackManager`.

## Parameters

- `p` - occurrence threshold above which a detection will be promoted to _confirmed_
- `r` - occurrence threshold below which a detection will be removed from management
