namespace cooperative_perception
{
template <typename DetectedObject, typename Track>
auto mahalanobis_distance(DetectedObject object, Track track) -> float
{
  return mahalanobis_distance(track.state, track.covariance, object.state);
}

template <typename DetectedObject, typename Track>
auto euclidean_distance(DetectedObject object, Track track) -> float
{
  return euclidean_distance(object.state, track.state);
}

}  // namespace cooperative_perception
