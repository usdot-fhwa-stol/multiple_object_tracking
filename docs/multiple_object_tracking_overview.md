# Multiple-object tracking overview

Pipelines designs vary depending on the application. The data flow we describe here is tailored to our needs but may
suffice for others. We integrate this library into a cooperative perception (CP) system where several actors perceive
the world and publish their detection information for others to use. Our pipeline comprises the following sequential
stages:

1. temporal alignment
2. track-to-detection scoring
3. track-to-detection association
4. track maintenance
5. track update

!!! info "Terminology"

    Here are some important terms to get us started. The exact definitions may differ across reference material.

    - **Detected object (detection)** - An object that our environment perception system detects. This is similar to a measurement in state-estimation pipelines.

    - **Tracked object (track)** - An object whose state information is being estimated and tracked by our system. This is similar to the current state estimate in state-estimation pipelines.

## Temporal alignment

Our tracking pipeline executes periodically, queueing received detections between iterations. Before the scoring,
association, and update steps, we must first propagate our queued detections' states to the current system time. This
ensures that all detections are aligned to the same time point. We will use these new states in the remaining stages.

Detections may move according to different motion models depending on their semantic class (vehicle, pedestrian,
cyclist, _etc._). The multiple-object tracking library current supports two motion models:

- constant turn-rate and velocity (CTRV)
- constant turn-rate and acceleration (CTRA)

## Track-to-detection scoring (and gating)

After aligning all detections to the current system time, we want to quantify how "close" each detection is to an
existing track. In this stage, we predict our existing tracks' states out to the current system time (the same time
point as our detections) then measure their distances to the detections. Our library provides several score functions
with support for user-programming to provide their own:

- Euclidean distance
- Mahalanobis distance

Unlike the detections, we don't keep the tracks' predicted states. We will update those later using some more
sophisticated methods.

### Gating

Sometimes, especially in sparsely-populated environments, we know when some track-to-detection associations are
impossible. We can remove sufficiently large scores in those scenarios to save some computation in the
track-to-detection association step.

!!! info

    Some multiple-object tracking system may combine gating and association stages.

## Track-to-detection association

This stage uses the scoring information to determine which (if any) detections should be associated with which tracks.
Solving the track-to-detection association problem is a whole research field. There are numerous algorithms that we
could use, each with varying degrees of complexity. We provide only one algorithm because it works well enough for our
application:

- Global nearest neighbors (GNN)

## Track maintenance



## Track update
