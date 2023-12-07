#ifndef MULTIPLE_OBJECT_TRACKING_DETAIL_DLIB_EXTENSIONS_HPP
#define MULTIPLE_OBJECT_TRACKING_DETAIL_DLIB_EXTENSIONS_HPP

#include <dlib/matrix/matrix.h>

namespace multiple_object_tracking::detail
{
template <typename Value>
struct PaddingValue
{
  Value value;
};

template <typename Value>
PaddingValue(Value) -> PaddingValue<Value>;

template <typename ValueType>
auto make_square_matrix(
  const dlib::matrix<ValueType> & matrix, const PaddingValue<ValueType> & padding_value) noexcept
  -> dlib::matrix<ValueType>
{
  const auto square_matrix_size{std::max(matrix.nr(), matrix.nc())};
  dlib::matrix<ValueType> square_matrix(square_matrix_size, square_matrix_size);

  for (auto row{0U}; row < matrix.nr(); ++row) {
    for (auto column{0U}; column < matrix.nc(); ++column) {
      square_matrix(row, column) = matrix(row, column);
    }
  }

  for (auto row{matrix.nr()}; row < square_matrix_size; ++row) {
    for (auto column{matrix.nc()}; column < square_matrix_size; ++column) {
      square_matrix(row, column) = padding_value.value;
    }
  }

  return square_matrix;
}

}  // namespace multiple_object_tracking::detail

#endif  // MULTIPLE_OBJECT_TRACKING_DETAIL_DLIB_EXTENSIONS_HPP
