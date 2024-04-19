# `multiple_object_tracking::euclidean_distance_score`

Defined in header `<multiple_object_tracking/scoring.hpp>`

## Call signature

```cpp
template <
    typename Track,
    typename Detection
> constexpr auto euclidean_distance_score(
    const Track & track,
    const Detection & detection
) -> std::optional<float>;
```

```cpp
template <
    typename... TrackAlternatives,
    typename... DetectionAlternatives
> constexpr auto euclidean_distance_score(
    const std::variant<TrackAlternatives...> & track,
    const std::variant<DetectionAlternatives...> & detection
) -> std::optional<float>;
```

Returns a score based on the Euclidean distance if `track` and `detection`
are comparible. Returns `std::nullopt` otherwise. Internally calls the
`multiple_object_tracking::euclidean_distance` function.

> [!NOTE]\
> This funciton-like entity is a [_niebloid_][niebloids_link], that is:
>
> - Explicit template argument lists cannot be specified when calling it.
> - It is invisible to
>   [argument-dependent lookup][argument_dependent_lookup_link].
> - When it is found by
>   [normal unqualified lookup][normal_unqualified_lookup_link] as the name to
>   the left of the function-call operator,
>   [argument-dependent lookup][argument_dependent_lookup_link] is inhibited.
>
> It is implemented as function a object.
>
> [niebloids_link]: https://brevzin.github.io/c++/2020/12/19/cpo-niebloid/
> [argument_dependent_lookup_link]: https://en.cppreference.com/w/cpp/language/adl
> [normal_unqualified_lookup_link]: https://en.cppreference.com/w/cpp/language/unqualified_lookup

## Parameters

- `track` - the track being scored
- `detection` - the detection being scored

## Return value

The Euclidean distance between the track and detection if they are comparable.
`std::nullopt` otherwise.

## Complexity

## Example

```cpp
// Incomplete
```

## See also
