# Key classes

## `DetectedObject`

This class represents a detected object received from the host agent (HA) or a remote agent (RA).

### Member variables

* `timestamp`: global time associated with the `state`
* `state`: state vector (CTRV model)
* `state_covariance`: covariance matrix associated with `state`

### Possible implementation

```cpp
struct {
    namespace cp = multiple_object_tracking;

    std::chrono::time_point timestamp;
    cp::CtrvState state;
    cp::CtrvStateCovariance covariance;
};
```

## `DetectedObjectList`

This class contains a list of `DetectedObject`s. We use a custom type instead of directly using a container type (e.g., `std::vector`) because we have size constraints to ensure timing requirements are met.

### Possible implementation

```cpp
using DetectedObjectList = boost::container::static_vector<DetectedObject, 200>;
```

## `RemoteAgent`

This class represents a remote agent that shares its own state and detected objects.

### Member variables

* `timestamp`: global time associated with the `state`
* `state`: state vector (CTRV model)
* `state_covariance`: covariance matrix associated with `state`

### Possible implementation

```cpp
struct {
    namespace cp = multiple_object_tracking;

    std::chrono::time_point timestamp;
    cp::CtrvState state;
    cp::CtrvStateCovariance covariance;
};
```
