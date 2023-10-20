# `multiple_object_tracking::Cluster<Detection>::get_centroid`

```cpp
[[nodiscard]]
auto get_centroid() const -> Point;
```

Returns the cluster's geometric center. If the cluster is empty, an exception of type `std::runtime_error` is thrown.

## Parameters

(none)

## Return value

The cluster's geometric center.

## Exceptions

`std::runtime_error` if cluster is empty.
