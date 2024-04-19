# `multiple_object_tracking::Detection`

```cpp
template <
    typename State,
    typename StateCovariance
> using Detection = multiple_object_tracking::DynamicObject<State, StateCovariance, struct DetectionTag{}>;
```

## Template parameters

- `State` - The (mathematical) vector representing the detection's state.
- `StateCovariance` - The matrix representing the detection's covariance.

## Specializations

| Type            | Definition                                  | Description                                                                              |
| --------------- | ------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `CtrvDetection` | `Detection<CtrvState, CtrvStateCovariance>` | a detection that moves according to the constant turn-rate and velocity (CTRV) model     |
| `CtraDetection` | `Detection<CtraState, CtraStateCovariance>` | a detection that moves according to the constant turn-rate and acceleration (CTRA) model |
