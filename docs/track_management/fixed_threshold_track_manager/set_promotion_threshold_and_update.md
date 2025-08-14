# `multiple_object_tracking::FixedThresholdTrackManager<Track>::set_promotion_threshold_and_update`

```cpp
auto set_promotion_threshold_and_update(const PromotionThreshold & threshold) noexcept -> void;
```

Sets the promotion threshold to a new value and updates the managed tracks' statuses based on the new threshold.

## Parameters

- `threshold` - occurrence threshold at or above which a detection will be promoted to _confirmed_
    NOTE: New detection for a confirmed track will force the occurrence to be at the threshold

## Return value

(none)
