#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct TopoOptSettings : psg::serialization::Serializable {
  Eigen::Vector3d lower_bound = Eigen::Vector3d(-0.2, -0.05, 0.5); // dummy
  Eigen::Vector3d upper_bound = Eigen::Vector3d(0.05, 0.2, 0.8); // dummy
  double neg_vol_res = kNegVolRes;
  double topo_res = kTopoRes;
  double attachment_size = kAttachmentSize;
  double vol_frac = kVolFrac;
  double contact_point_size = kContactPointSize;
  double base_thickness = kBaseThickness;

  DECL_SERIALIZE() {
    constexpr int version = 2;
    SERIALIZE(version);
    SERIALIZE(lower_bound);
    SERIALIZE(upper_bound);
    SERIALIZE(neg_vol_res);
    SERIALIZE(topo_res);
    SERIALIZE(attachment_size);
    SERIALIZE(vol_frac);
    SERIALIZE(contact_point_size);
    SERIALIZE(base_thickness);
  }

  DECL_DESERIALIZE() {
    int __unused;
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(lower_bound);
      DESERIALIZE(upper_bound);
      DESERIALIZE(neg_vol_res);
      DESERIALIZE(topo_res);
      DESERIALIZE(attachment_size);
      DESERIALIZE(__unused); // int
      DESERIALIZE(contact_point_size);
      DESERIALIZE(base_thickness);
    } else if (version == 2) {
      DESERIALIZE(lower_bound);
      DESERIALIZE(upper_bound);
      DESERIALIZE(neg_vol_res);
      DESERIALIZE(topo_res);
      DESERIALIZE(attachment_size);
      DESERIALIZE(vol_frac);
      DESERIALIZE(contact_point_size);
      DESERIALIZE(base_thickness);
    }
  }
};

inline std::ostream& operator<<(std::ostream& f, const TopoOptSettings& c) {
  f << "TopoOptSettings:\n"
    << "  neg_vol_res: " << c.neg_vol_res << "\n"
    << "  topo_res: " << c.topo_res << "\n"
    << "  vol_frac: " << c.vol_frac << std::endl;
  return f;
}

}  // namespace psg
