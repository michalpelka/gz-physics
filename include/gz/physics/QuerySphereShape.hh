/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_PHYSICS_QUERYSPHERESHAPE_HH_
#define GZ_PHYSICS_QUERYSPHERESHAPE_HH_

#include <vector>

#include <gz/physics/Feature.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/Shape.hh>

namespace gz
{
namespace physics
{
/// \brief QuerySphereShapeFeature is a feature for finding all collision
/// shapes that overlap with a given sphere in world coordinates.
/// Unlike GetContactsFromLastStepFeature, this query is not tied to a
/// simulation step and can be called at any time.
class GZ_PHYSICS_VISIBLE QuerySphereShapeFeature
    : public virtual Feature
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<LinearVector>;
    public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;

    /// \brief Result of a single sphere intersection hit.
    public: struct SphereHit
    {
      /// \brief The colliding shape.
      ShapePtrType shape;
      /// \brief Contact normal on the hit surface, in world frame,
      /// pointing away from the surface toward the query sphere center.
      VectorType normal;
      /// \brief Contact point on the hit surface in world frame.
      VectorType point;
      /// \brief Penetration depth (positive = overlapping).
      Scalar depth{0};
    };

    /// \brief Find all collision shapes that overlap with a sphere.
    /// \param[in] _center Center of the sphere in world coordinates.
    /// \param[in] _radius Radius of the sphere in meters.
    /// \return One SphereHit per overlapping shape (deduplicated).
    public: std::vector<SphereHit> QuerySphereIntersections(
        const VectorType &_center, Scalar _radius) const;
  };

  /// \brief Plain-data hit returned by the Implementation layer.
  public: template <typename PolicyT>
  struct HitData
  {
    using Scalar = typename PolicyT::Scalar;
    using VectorType =
        typename FromPolicy<PolicyT>::template Use<LinearVector>;

    Identity shapeID;
    VectorType normal;
    VectorType point;
    Scalar depth{0};
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType =
        typename FromPolicy<PolicyT>::template Use<LinearVector>;

    /// \brief Find collisions overlapping a sphere and return hit data.
    public: virtual std::vector<HitData<PolicyT>> QuerySphereIntersections(
        const Identity &_worldID,
        const VectorType &_center,
        Scalar _radius) const = 0;
  };
};

}  // namespace physics
}  // namespace gz

#include "gz/physics/detail/QuerySphereShape.hh"

#endif