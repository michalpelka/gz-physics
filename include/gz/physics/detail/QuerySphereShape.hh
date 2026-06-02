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

#ifndef GZ_PHYSICS_DETAIL_QUERYSPHERESHAPE_HH_
#define GZ_PHYSICS_DETAIL_QUERYSPHERESHAPE_HH_

#include <gz/physics/QuerySphereShape.hh>

namespace gz
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto QuerySphereShapeFeature::World<PolicyT, FeaturesT>::QuerySphereIntersections(
    const VectorType &_center,
    Scalar _radius) const -> std::vector<SphereHit>
{
  auto hits =
      this->template Interface<QuerySphereShapeFeature>()
          ->QuerySphereIntersections(this->identity, _center, _radius);

  std::vector<SphereHit> output;
  output.reserve(hits.size());
  for (auto &h : hits)
  {
    SphereHit hit;
    hit.shape  = ShapePtrType(this->pimpl, h.shapeID);
    hit.normal = h.normal;
    hit.point  = h.point;
    hit.depth  = h.depth;
    output.push_back(std::move(hit));
  }
  return output;
}

}  // namespace physics
}  // namespace gz

#endif