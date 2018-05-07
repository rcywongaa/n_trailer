#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_bool.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"

struct VisualStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 3;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kHeading = 2;

  /// Returns a vector containing the names of each coordinate within this
  /// class. The indices within the returned vector matches that of this class.
  /// In other words, `VisualStateIndices::GetCoordinateNames()[i]`
  /// is the name for `BasicVector::GetAtIndex(i)`.
  static const std::vector<std::string>& GetCoordinateNames();
};

/// Specializes BasicVector with specific getters and setters.
template <typename T>
class VisualState : public drake::systems::BasicVector<T> {
 public:
  /// An abbreviation for our row index constants.
  typedef VisualStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  VisualState() : drake::systems::BasicVector<T>(K::kNumCoordinates) {
    this->SetFromVector(drake::VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// Create a symbolic::Variable for each element with the known variable
  /// name.  This is only available for T == symbolic::Expression.
  template <typename U = T>
  typename std::enable_if<std::is_same<U, drake::symbolic::Expression>::value>::type
  SetToNamedVariables() {
    this->set_x(drake::symbolic::Variable("x"));
    this->set_y(drake::symbolic::Variable("y"));
    this->set_heading(drake::symbolic::Variable("heading"));
  }

  VisualState<T>* DoClone() const override { return new VisualState; }

  /// @name Getters and Setters
  //@{
  /// x
  const T& x() const { return this->GetAtIndex(K::kX); }
  void set_x(const T& x) { this->SetAtIndex(K::kX, x); }
  /// y
  const T& y() const { return this->GetAtIndex(K::kY); }
  void set_y(const T& y) { this->SetAtIndex(K::kY, y); }
  /// heading
  const T& heading() const { return this->GetAtIndex(K::kHeading); }
  void set_heading(const T& heading) { this->SetAtIndex(K::kHeading, heading); }
  //@}

  /// See VisualStateIndices::GetCoordinateNames().
  static const std::vector<std::string>& GetCoordinateNames() {
    return VisualStateIndices::GetCoordinateNames();
  }

  /// Returns whether the current values of this vector are well-formed.
  drake::Bool<T> IsValid() const {
    using std::isnan;
    auto result = (T(0) == T(0));
    result = result && !isnan(x());
    result = result && !isnan(y());
    result = result && !isnan(heading());
    return result;
  }
};
