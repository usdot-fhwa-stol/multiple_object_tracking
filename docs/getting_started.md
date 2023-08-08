# Getting started

There are five steps in the multiple-object tracking pipeline

1. **Temporal alignment:** Propagates incoming detections' states to the system's current time.
2. **Track-to-detection scoring (gating optional):** Quantifies how "close" a detection is to an existing track.
   Closeness depends on the scoring metric.
3. **Association:** Maps a detection to a specific track.
4. **Track maintenance:** Adds and removes tracks from a maintained list. Also promotes and demotes track statuses.
5. **Track-to-detection fusion:** Fuses detections' data with their associated tracks to improve the tracks' state
   estimates.

You will most-likely want to run each step sequentially. The library currently does not have a single do-it-all
function, so you will have to individually call the functions.

## Temporal alignment

The main function for this step is `cooperative_perception::propagate_to_time()`, which takes a detection and a time.
The function will propagate the detection's state to the time point using the detection's motion model.

## Track-to-detection scoring (and gating)

The main function for this step is `cooperative_perception::score_track_and_detections()`, which takes in a collection
of tracks, a collection of detections, and a scoring metric. The function returns a one-to-many map between track UUIDs
and detection UUIDs.

An optional sub-step involves gating detections that are beyond some threshold. These outliers will be dropped from a
track's scoring map.

## Association

The main function for this step is `cooperative_perception::associate_detections_to_tracks()`, which takes in the
scoring map and an association strategy. The function returns an association map, mapping a single track UUID to one or
more detection UUIDs.

## Track maintenance

The main entity for this step is the `TrackManager`, which maintains the list of tracks and their statuses according to
several policies. In each pipeline step, the manager can update existing tracks' statuses from the association mapping,
and it can add newly-generated tracks.

## Track-to-detection fusion

The main function in this step is `cooperative_perception::fuse_detection_to_track()`, which combines the state
vectors between a track and its associated detection(s). The function takes in a single track, one (or more) detections,
and a fusion strategy.

# A full example

The following code sample shows how you can compose the tracking step functions to make a full pipeline. **Note:** Some
functions (such as `get_system_time()`) are conceptual and don't exist in the library.

```cpp
using DetectionType = // ... (Depends on your application)
using TrackType = // ... (Depends on your application)

namespace cp = cooperative_perception;

auto execute_pipeline(cp::TrackManager& track_manager, const std::vector<DetectionType>& detections) -> void {
    const auto system_time = get_system_time();

    // Temporal alignment
    for (auto& detection : detections) {
        cp::propagate_to_time(detection, system_time);
    }

    // Scoring (no gating in this example)
    auto predicted_tracks = track_manager.get_tracks();
    for (auto& track : predicted_tracks) {
        cp::propagate_to_time(track, system_time);
    }

    const auto scores = cp::score_tracks_and_detections(predicted_tracks, detections, cp::mahalanobis_distance);

    // Association
    const auto tracks = track_manager.get_tracks();
    const auto associations = cp::associate_detections_to_tracks(scores, cp::gnn_association);
    const auto associated_track_uuids = cp::get_uuids_if(tracks, cp::is_associated_track{ associations });

    std::vector<DetectionType> unassociated_detections;
    std::copy_if(std::cbegin(detections), std::cend(detections), std::back_inserter(unassociated_detections),
                 std::not_fn(cp::is_associated_detection{ associations }));

    // Track maintenance
    const auto new_tracks = cp::make_tracks<cp::TrackType>(unassociated_detections);

    track_manager.update_track_statuses(associations);
    track_manager.add_new_tracks(new_tracks);

    // Detection-to-track fusion
    std::unordered_map<std::string, DetectionType> detection_uuid_map;
    for (auto& track : track_manger.get_tracks()) {
        const auto detection_uuids = associations[cp::get_uuid(track)];

        for (const auto& uuid : detection_uuids) {
            cp::fuse_detection_to_track(detection_uuid_map[uuid], track);
        }
    }
}

auto main() -> int {
    cp::TrackManager track_manager;

    while (!should_stop()) {
        execute_pipeline(track_manager, get_detections());
        wait_for_some_event();
    }

    return 0;
}
```

# Customization

The library provides several areas for customization, particularly with the motion models. Most of the library's
functions are function templates, so they should work with your custom motion models. See below for details.

Other customization points include scoring metrics, gating thresholds, association strategies, and fusion methods.

## Bring your own detection and track types

The `Detection` and `Track` class templates depend on a motion model. Users are free to specify their own models, and
we provide some implementations to help you get started. Many of the library's functions operate on detections, tracks,
or both. We implemented these functions as function templates so that you can specify custom `Detection` and `Track`
types. For the rest of this guide, we will use the term `DetectionType` and `TrackType` to generically refer to
detection and track types, respectively.

Some use cases involve tracking several objects with differing motion models. You may have one motion model for
vehicles and another for pedestrians. In other applications, all detections and tracks use the same motion model. Our
library supports both scenarios. If you need to handle several motion models in your application, we recommend using
`std::variant`s as your `DetectionType` and `TrackType`. For example,

```cpp
namespace cp = cooperative_perception;

using DetectionType = std::variant<cp::CtrvDetection, cp::CtraDetection>;
using TrackType = std::variant<cp::CtrvTrack, cp::CtraTrack>;

std::vector<DetectionType> detections = get_detections();  // This is an imaginary function.
```

If you use only one motion model in your application, we recommend you define `DetectionType` as that model. For
example,

```cpp
namespace cp = cooperative_perception;

using DetectionType = cp::CtrvDetection;
using TrackType = cp::CtrvTrack;

std::vector<DetectionType> detections = get_detections();  // This is an imaginary function.
```
