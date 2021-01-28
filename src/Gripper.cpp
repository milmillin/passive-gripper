#include "Gripper.h"

#include <Eigen/Geometry>
#include <dxflib/dxflib>

#include "Geometry.h"
#include "MeshInfo.h"

namespace gripper {

const float ANGLE_TO_RADIAN = EIGEN_PI / 180;

Gripper::Gripper(const Eigen::MatrixXd& mesh_V,
                 const Eigen::MatrixXi& mesh_F,
                 const std::vector<Eigen::Vector3d>& contactPoints,
                 double rodDiameter,
                 const Eigen::Vector2f& grabAngle)
    : m_grabAngle(grabAngle * ANGLE_TO_RADIAN) {
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.rotate(Eigen::AngleAxisd(-m_grabAngle(0), Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(m_grabAngle(1), Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d tInverse = t.inverse();

  // Rotated mesh so that the gripper comes in -x direction
  Eigen::MatrixXd rotated_V(mesh_V.rows(), 3);
  for (size_t i = 0; i < mesh_V.rows(); i++) {
    Vector3d v = mesh_V.row(i);
    rotated_V.row(i) = tInverse * v;
  }

  MeshInfo info(rotated_V, mesh_F);

  gripper_V.resize(8 + cylinderNumV * contactPoints.size(), 3);
  gripper_F.resize(12 + cylinderNumF * contactPoints.size(), 3);

  // Generate backplate
  static const double padding = 0.075; // TODO
  m_plateDimension =
      Eigen::Vector2d(info.size.z() + padding * 2, info.size.y() + padding * 2);

  gripper_V.block<8, 3>(0, 0) = GenerateCubeV(
      Eigen::Vector3d(info.maximum.x(), info.minimum.y() - padding, info.minimum.z() - padding),
      // TODO: Check backplate thickness
      Eigen::Vector3d(0.02, m_plateDimension.y(), m_plateDimension.x()));
  gripper_F.block<12, 3>(0, 0) = cube_F;

  // Generate rods
  for (size_t i = 0; i < contactPoints.size(); i++) {
    Vector3d contactPosition = tInverse * contactPoints[i];
    Vector3d origin(info.maximum.x(), contactPosition.y(), contactPosition.z());

    m_rodLocations.push_back(
        Eigen::Vector2d(contactPosition.z() - info.minimum.z() + padding,
                        contactPosition.y() - info.minimum.y() + padding));
    m_rodLengths.push_back(info.maximum.x() - contactPosition.x());

    gripper_V.block<cylinderNumV, 3>(8 + i * cylinderNumV, 0) =
        GenerateCylinderV(origin, contactPosition, rodDiameter / 2);
    gripper_F.block<cylinderNumF, 3>(12 + i * cylinderNumF, 0) =
        cylinder_F.array() + (8 + i * cylinderNumV);
  }

  // Rotate back
  for (size_t i = 0; i < gripper_V.rows(); i++) {
    Vector3d v = gripper_V.row(i);
    gripper_V.row(i) = t * v;
  }
}

Gripper::Gripper() {}

void Gripper::WriteDXF(const std::string& filename) const {
  DL_Dxf dxf;
  std::unique_ptr<DL_WriterA> dw(dxf.out(filename.c_str(), DL_Codes::AC1015));
  if (dw == nullptr) {
    std::cout << "Cannot open file " + filename << std::endl;
    return;
  }

  // section header:
  dxf.writeHeader(*dw);
  dw->sectionEnd();

  // section tables:
  dw->sectionTables();

  // VPort
  dxf.writeVPort(*dw);

  // LTYPE:
  dw->tableLinetypes(1);
  dxf.writeLinetype(*dw,
                    DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
  dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
  dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
  dw->tableEnd();

  // LAYER:
  dw->tableLayers(1);
  dxf.writeLayer(*dw,
                 DL_LayerData("0", 0),
                 DL_Attributes("", DL_Codes::white, 100, "CONTINUOUS", 1));
  dw->tableEnd();

  // STYLE:
  dw->tableStyle(1);
  DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
  style.bold = false;
  style.italic = false;

  dxf.writeStyle(*dw, style);
  dw->tableEnd();

  dxf.writeView(*dw);
  dxf.writeUcs(*dw);

  dw->tableAppid(1);
  dxf.writeAppid(*dw, "ACAD");
  dw->tableEnd();

  // DIMSTYLE:
  dxf.writeDimStyle(*dw, 2.5, 0.625, 0.625, 0.625, 2.5);

  // BLOCK_RECORD:
  dxf.writeBlockRecord(*dw);
  dw->tableEnd();

  dw->sectionEnd();

  dw->sectionBlocks();
  dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
  dxf.writeEndBlock(*dw, "*Model_Space");
  dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
  dxf.writeEndBlock(*dw, "*Paper_Space");
  dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
  dxf.writeEndBlock(*dw, "*Paper_Space0");

  dw->sectionEnd();

  dw->sectionEntities();
  // write all your entities..

  DL_Attributes defaultAttribute("0", 256, -1, "BYLAYER", 1);

  double width = m_plateDimension.x() * 1000;
  double height = m_plateDimension.y() * 1000;

  dxf.writeLine(
      *dw, DL_LineData(0, 0, 0, 0, height, 0), defaultAttribute);
  dxf.writeLine(*dw,
                DL_LineData(0,
                            height,
                            0,
                            width,
                            height,
                            0),
                defaultAttribute);
  dxf.writeLine(*dw,
                DL_LineData(width,
                            height,
                            0,
                            width,
                            0,
                            0),
                defaultAttribute);
  dxf.writeLine(
      *dw, DL_LineData(width, 0, 0, 0, 0, 0), defaultAttribute);

  // TODO: Change settings
  // fixed for 12mm shaft
  // https://www.mcmaster.com/9604T14/
  static const double shaftRadius = 6;     // in mm
  static const double mountRadius = 19.5;  // in mm
  static const double screwRadius = 2.2;   // in mm

  for (size_t i = 0; i < m_rodLocations.size(); i++) {
    dxf.writeCircle(*dw,
                    DL_CircleData(m_rodLocations[i].x() * 1000,
                                  m_rodLocations[i].y() * 1000,
                                  0,
                                  shaftRadius),
                    defaultAttribute);

    for (int j = 0; j < 3; j++) {
      double angle = EIGEN_PI * 2 / 3 * j;
      Eigen::Vector2d screwLocation =
          m_rodLocations[i] * 1000 +
          mountRadius * Eigen::Vector2d(cos(angle), sin(angle));
      dxf.writeCircle(
          *dw,
          DL_CircleData(screwLocation.x(), screwLocation.y(), 0, screwRadius),
          defaultAttribute);
    }
  }

  dw->sectionEnd();

  dxf.writeObjects(*dw);
  dxf.writeObjectsEnd(*dw);

  dw->dxfEOF();
  dw->close();
}

}  // namespace gripper