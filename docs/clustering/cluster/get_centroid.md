# `multiple_object_tracking::Cluster<Detection>::get_centroid`

```cpp
[[nodiscard]]
auto get_centroid() const -> Point;
```

Returns the cluster's _centroid_: a point equal to the mean of a cluster's contained members. If the cluster is empty,
an exception of type `std::runtime_error` is thrown.

## Parameters

(none)

## Return value

The cluster's centroid.

## Exceptions

`std::runtime_error` if cluster is empty.
