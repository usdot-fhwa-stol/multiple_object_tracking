# `multiple_object_tracking::HasAssociation::operator()`

```cpp
template <typename Object>
[[nodiscard]]
auto operator()(const Object & object) const noexcept -> bool;
```

Checks if the specified object has an association within the `HasAssociation`'s
internal association map.

## Parameters

- `object` - the object that will be checks

## Return value

`true` if the object has an association; `false` otherwise.
