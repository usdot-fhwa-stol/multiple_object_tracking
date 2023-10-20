# `multiple_object_tracking::Point`

Defined in header `<multiple_object_tracking/clustering.hpp>`

```cpp
struct Point;
```

This class represents a cluster's _centroid_: a point equal to the mean of a cluster's contained members. It is a
sub-vector of `multiple_object_tracking::CtrvState` and `multiple_object_tracking::CtraState` containing the common
(mathematical) vector elements.

## Data members

| Name         | Type                                            | Description                                          |
| ------------ | ----------------------------------------------- | ---------------------------------------------------- |
| `position_x` | `units::length::meter_t`                        | _x_-position                                         |
| `position_y` | `units::length::meter_t`                        | _y_-position                                         |
| `velocity`   | `units::velocity::meters_per_second_t`          | signed speed in direction of yaw                     |
| `yaw`        | `multiple_object_tracking::Angle`               | angle from the coordinate frame's _x_-axis           |
| `yaw_rate`   | `units::angular_velocity::radians_per_second_t` | angular signed speed (counter-clockwise is positive) |

## Non-member functions

|                                    |                          |
| ---------------------------------- | ------------------------ |
| [`operator/`][operator_divide_doc] | performs scalar division |

[operator_divide_doc]: operator_divide.md
