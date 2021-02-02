#include "Gripper.h"

#include <Eigen/Geometry>
#include <dxflib/dxflib>

#include "Geometry.h"
#include "MeshInfo.h"

namespace gripper {

Gripper::Gripper(const Eigen::MatrixXd& mesh_V,
                 const Eigen::MatrixXi& mesh_F,
                 const std::vector<ContactPoint>& contactPoints,
                 const Eigen::Vector3d& centerOfMass,
                 const VoxelPipelineSettings& settings)
    : m_grabAngle(settings.grabAngle * DEGREE_TO_RADIAN),
      m_rodRadius(settings.rodDiameter / 2),
      m_fitterRadius(settings.fitterDiameter / 2),
      m_fitterMountRadius(settings.fitterMountDiameter / 2),
      m_fitterScrewRadius(settings.fitterScrewDiameter / 2) {
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.rotate(Eigen::AngleAxisd(-m_grabAngle(0), Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(m_grabAngle(1), Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d tInverse = t.inverse();

  // Rotated mesh so that the gripper comes in -x direction
  Eigen::MatrixXd rotated_V(mesh_V.rows(), 3);
  for (size_t i = 0; i < mesh_V.rows(); i++) {
    Eigen::Vector3d v = mesh_V.row(i);
    rotated_V.row(i) = tInverse * v;
  }

  MeshInfo info(rotated_V, mesh_F);

  gripper_V.resize(8 + cylinderNumV * contactPoints.size(), 3);
  gripper_F.resize(12 + cylinderNumF * contactPoints.size(), 3);

  m_rodLocations.resize(contactPoints.size(), Eigen::NoChange);

  Eigen::Vector3d rotatedCenterOfMass = tInverse * centerOfMass;
  Eigen::RowVector2d projectedCenterOfMass(rotatedCenterOfMass.z(),
                                           rotatedCenterOfMass.y());

  // Generate rods
  for (size_t i = 0; i < contactPoints.size(); i++) {
    Eigen::Vector3d contactPosition = tInverse * contactPoints[i].position;
    Eigen::Vector3d origin(
        info.maximum.x(), contactPosition.y(), contactPosition.z());

    m_rodLocations.row(i) =
        Eigen::RowVector2d(contactPosition.z(), contactPosition.y());
    m_rodLengths.push_back(info.maximum.x() - contactPosition.x());

    gripper_V.block<cylinderNumV, 3>(8 + i * cylinderNumV, 0) =
        GenerateCylinderV(origin, contactPosition, settings.rodDiameter / 2);
    gripper_F.block<cylinderNumF, 3>(12 + i * cylinderNumF, 0) =
        cylinder_F.array() + (8 + i * cylinderNumV);
  }

  // Generate backplate
  Eigen::RowVector2d padding(m_fitterRadius + 0.005, m_fitterRadius + 0.005);
  Eigen::RowVector2d minCoord =
      m_rodLocations.colwise().minCoeff() - padding;
  Eigen::RowVector2d maxCoord =
      m_rodLocations.colwise().maxCoeff() + padding;

  // Add the arm mount 40mm by 40mm at the bottom
  // The x of the center hole is the x of projected center of mass
  // The y of the center hole is 9mm below the 3 rods boundary.
  minCoord.x() = std::min(minCoord.x(), projectedCenterOfMass.x() - 0.02);
  maxCoord.x() = std::max(maxCoord.x(), projectedCenterOfMass.x() + 0.02);
  minCoord.y() -= 0.04;

  m_plateDimension = (maxCoord - minCoord).transpose();
  m_rodLocations -= minCoord.replicate(m_rodLocations.rows(), 1);
  m_cmLocationX = projectedCenterOfMass.x() - minCoord.x();

  // TODO: Check backplate thickness
  gripper_V.block<8, 3>(0, 0) = GenerateCubeV(
      Eigen::Vector3d(info.maximum.x(), minCoord.y(), minCoord.x()),
      Eigen::Vector3d(0.02, m_plateDimension.y(), m_plateDimension.x()));
  gripper_F.block<12, 3>(0, 0) = cube_F;

  // Rotate back
  for (size_t i = 0; i < gripper_V.rows(); i++) {
    Eigen::Vector3d v = gripper_V.row(i);
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

  // Drawing starts here

  // Plate
  dxf.writePolyline(*dw, DL_PolylineData(4, 0, 0, 1), defaultAttribute);
  dxf.writeVertex(*dw, DL_VertexData(0, 0));
  dxf.writeVertex(*dw, DL_VertexData(0, height));
  dxf.writeVertex(*dw, DL_VertexData(width, height));
  dxf.writeVertex(*dw, DL_VertexData(width, 0));
  dxf.writePolylineEnd(*dw);

  // Contact fitter
  for (ssize_t i = 0; i < m_rodLocations.rows(); i++) {
    dxf.writeCircle(*dw,
                    DL_CircleData(m_rodLocations.row(i).x() * 1000,
                                  m_rodLocations.row(i).y() * 1000,
                                  0,
                                  m_rodRadius * 1000),
                    defaultAttribute);

    for (int j = 0; j < 3; j++) {
      double angle = EIGEN_PI * 2 / 3 * j;
      Eigen::RowVector2d screwLocation =
          m_rodLocations.row(i) * 1000 +
          (m_fitterMountRadius * 1000) * Eigen::RowVector2d(cos(angle), sin(angle));
      dxf.writeCircle(
          *dw,
          DL_CircleData(
              screwLocation.x(), screwLocation.y(), 0, m_fitterScrewRadius * 1000),
          defaultAttribute);
    }

    // Mount
    double center = m_cmLocationX * 1000; // mm
    dxf.writeCircle(
        *dw, DL_CircleData(center, 31, 0, 5.8), defaultAttribute);
    dxf.writeArc(*dw,
                 DL_ArcData(center - 12.5, 7, 0, 2.5, 0, 180),
                 defaultAttribute);
    dxf.writeArc(*dw,
                 DL_ArcData(center - 12.5, 5, 0, 2.5, 180, 0),
                 defaultAttribute);
    dxf.writeLine(*dw,
                  DL_LineData(center - 15, 5, 0, center - 15, 7, 0),
                  defaultAttribute);
    dxf.writeLine(*dw,
                  DL_LineData(center - 10, 5, 0, center - 10, 7, 0),
                  defaultAttribute);

    dxf.writeArc(*dw,
                 DL_ArcData(m_cmLocationX * 1000 + 11.5, 6, 0, 2.5, 90, 270),
                 defaultAttribute);
    dxf.writeArc(*dw,
                 DL_ArcData(m_cmLocationX * 1000 + 13.5, 6, 0, 2.5, 270, 90),
                 defaultAttribute);
    dxf.writeLine(*dw,
                  DL_LineData(center + 11.5, 8.5, 0, center + 13.5, 8.5, 0),
                  defaultAttribute);
    dxf.writeLine(*dw,
                  DL_LineData(center + 11.5, 3.5, 0, center + 13.5, 3.5, 0),
                  defaultAttribute);

  }

  dw->sectionEnd();

  dxf.writeObjects(*dw);
  dxf.writeObjectsEnd(*dw);

  dw->dxfEOF();
  dw->close();
}

}  // namespace gripper