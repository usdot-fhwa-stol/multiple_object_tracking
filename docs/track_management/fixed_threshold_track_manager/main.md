# `multiple_object_tracking::FixedThresholdTrackManager`

Defined in header `<multiple_object_tracking/track_management.hpp>`

```cpp
template <
    typename Track
> class FixedThresholdTrackManager;
```

## Template parameters

- `Track` - The track type being managed.

## Member functions

|                                  |                                             |
| -------------------------------- | ------------------------------------------- |
| [(constructor)][constructor_doc] | constructs the `FixedThresholdTrackManager` |
| [(destructor)][destructor_doc]   | destructs the `FixedThresholdTrackManager`  |

[constructor_doc]: constructor.md
[destructor_doc]: destructor.md

### Track access

|                                                    |                                                          |
| -------------------------------------------------- | -------------------------------------------------------- |
| [`get_tentative_tracks`][get_tentative_tracks_doc] | access a list of tracks whose status is _tentative_      |
| [`get_confirmed_tracks`][get_confirmed_tracks_doc] | access a list of tracks whose status is _confirmed_      |
| [`get_all_tracks`][get_all_tracks_doc]             | access a list of all managed tracks regardless of status |

[get_tentative_tracks_doc]: get_tentative_tracks.md
[get_confirmed_tracks_doc]: get_confirmed_tracks.md
[get_all_tracks_doc]: get_all_tracks.md

### Modifiers

|                                                  |                                                                         |
| ------------------------------------------------ | ----------------------------------------------------------------------- |
| [`update_track_lists`][update_track_lists_doc]   | update each track's status based on the detection-to-track associations |
| [`add_tentative_track`][add_tentative_track_doc] | add a track to be managed                                               |

[update_track_lists_doc]: update_track_lists.md
[add_tentative_track_doc]: add_tentative_track.md
