#ifndef COOPERATIVE_PERCEPTION_REMOVE_CVREF_HPP
#define COOPERATIVE_PERCEPTION_REMOVE_CVREF_HPP

#include <type_traits>

namespace cooperative_perception
{
template <typename T>
struct remove_cvref
{
  using type = std::remove_cv_t<std::remove_reference_t<T>>;
};

template <typename T>
using remove_cvref_t = typename remove_cvref<T>::type;

}  // namespace cooperative_perception

#endif  // COOPERATIVE_PERCEPTION_REMOVE_CVREF_HPP
