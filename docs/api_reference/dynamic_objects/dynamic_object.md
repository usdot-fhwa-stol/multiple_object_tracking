# `multiple_object_tracking::DynamicObject`

Defined in header `<multiple_object_tracking/dynamic_object.hpp>`

```cpp
template <
    typename State,
    typename StateCovariance,
    typename Tag
> class DynamicObject;
```

## Template parameters

- `State` - The (mathematical) vector representing the object's state.
- `StateCovariance` - The matrix representing the object's covariance.
- `Tag` - The semantic tag representing the `class`'s purpose.

## Data members

| Name         | Type                             | Description                              |
| ------------ | -------------------------------- | ---------------------------------------- |
| `timestamp`  | `units::time::second_t`          | measurement timestamp                    |
| `state`      | `State`                          | state vector at time of measurement      |
| `covariance` | `StateCovariance`                | covariance matrix at time of measurement |
| `uuid`       | `multiple_object_tracking::Uuid` | universally-unique identifier (UUID)     |

## Specializations

|                                                        |     |
| ------------------------------------------------------ | --- |
| [`multiple_object_tracking::Detection`][detection_doc] |     |
| [`multiple_object_tracking::Track`][track_doc]         |     |

## Non-member functions

|                                      |                                                          |
| ------------------------------------ | -------------------------------------------------------- |
| [`get_timestamp`][get_timestamp_doc] | obtains the dynamic object's timestamp                   |
| [`get_uuid`][get_uuid_doc]           | obtains the dynamic object's UUID                        |
| [`make_track`][make_track_doc]       | creates a [`multiple_object_tracking::Track`][track_doc] |

[detection_doc]: detection.md
[track_doc]: track.md
[get_timestamp_doc]: get_timestamp.md
[get_uuid_doc]: get_uuid.md
[make_track_doc]: make_track.md
