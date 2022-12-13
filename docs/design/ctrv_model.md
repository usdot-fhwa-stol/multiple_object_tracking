# Constant turn-rate and velocity (CTRV) model

## `CtrvState`

This class contains the state variables used for the constant turn-rate and velocity (CTRV) motion model.

### Possible implementation

```cpp
using CtrvState = Eigen::Matrix<float, 5, 1>;
```


## `CtrvCovariance`

This class contains the covariance matrix for the state vector used in the CTRV motion model.

### Possible implementation

```cpp
using CtrvCovariance = Eigen::Matrix<float, 5, 5>;
```

## Calculate next CTRV state

### Interface
```cpp
auto nextState(
    const cooperative_perception::CtrvState& state,
    const cooperative_perception::CtrvStateCovariance& covariance,
    float time_step
) -> std::tuple<cooperative_perception::CtrvState, cooperative_perception::CtrvStateCovariance>;
```

### Parameters
* `state`: current system state
* `covariance`: current system covariance
* `time_step`: state propagation duration

### Returns
* `std::tuple` containing the new `CtrvState` and `CtrvStateCovariance`
