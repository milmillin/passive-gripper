#include "Gripper.h"

#include <dxflib/dxflib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

namespace gripper {

Gripper::Gripper(const std::vector<ContactPoint>& contactPoints,
                 const Eigen::Vector3d& centerOfMass,
                 const VoxelPipelineSettings& settings,
                 const MeshInfo& meshInfo,
                 const Eigen::Affine3d& rotation)
    : m_rodRadius(settings.rodDiameter / 2),
      m_rodClearance(settings.rodClearance),
      m_fitterRadius(settings.fitterDiameter / 2),
      m_fitterMountRadius(settings.fitterMountDiameter / 2),
      m_fitterScrewRadius(settings.fitterScrewDiameter / 2) {
  gripper_V.resize(8 + cylinderNumV * contactPoints.size(), 3);
  gripper_F.resize(12 + cylinderNumF * contactPoints.size(), 3);

  rodLocations.resize(contactPoints.size(), Eigen::NoChange);

  Eigen::Vector3d rotatedCenterOfMass = rotation * centerOfMass;
  Eigen::RowVector2d projectedCenterOfMass(rotatedCenterOfMass.z(),
                                           rotatedCenterOfMass.y());

  // Generate rods
  for (size_t i = 0; i < contactPoints.size(); i++) {
    Eigen::Vector3d contactPosition = rotation * contactPoints[i].position;
    Eigen::Vector3d origin(
        meshInfo.maximum.x(), contactPosition.y(), contactPosition.z());

    rodLocations.row(i) =
        Eigen::RowVector2d(contactPosition.z(), contactPosition.y());
    rodLengths.push_back(meshInfo.maximum.x() - contactPosition.x());

    gripper_V.block<cylinderNumV, 3>(8 + i * cylinderNumV, 0) =
        GenerateCylinderV(origin, contactPosition, settings.rodDiameter / 2);
    gripper_F.block<cylinderNumF, 3>(12 + i * cylinderNumF, 0) =
        cylinder_F.array() + (8 + i * cylinderNumV);
  }

  if (contactPoints.empty()) {
    rodLocations.resize(1, 2);
    rodLocations(0) = 0;
    rodLocations(1) = 0;
  }

  // Generate backplate
  Eigen::RowVector2d padding(m_fitterRadius + 0.005, m_fitterRadius + 0.005);
  Eigen::RowVector2d minCoord = rodLocations.colwise().minCoeff() - padding;
  Eigen::RowVector2d maxCoord = rodLocations.colwise().maxCoeff() + padding;

  // Add the arm mount 40mm by 42mm at the top
  // The x of the center hole is the x of projected center of mass
  // The y of the center hole is 9mm below the 3 rods boundary.
  m_mountOriginY = maxCoord.y() - minCoord.y();
  minCoord.x() = std::min(minCoord.x(), projectedCenterOfMass.x() - 0.02);
  maxCoord.x() = std::max(maxCoord.x(), projectedCenterOfMass.x() + 0.02);
  maxCoord.y() += 0.042;

  plateDimension = (maxCoord - minCoord).transpose();
  rodLocations -= minCoord.replicate(rodLocations.rows(), 1);
  m_cmLocationX = projectedCenterOfMass.x() - minCoord.x();
  plateOrigin =
      Eigen::Vector3d(meshInfo.maximum.x(), minCoord.y(), minCoord.x());

  // TODO: Check backplate thickness
  gripper_V.block<8, 3>(0, 0) = GenerateCubeV(
      plateOrigin,
      Eigen::Vector3d(plateThickness, plateDimension.y(), plateDimension.x()));
  gripper_F.block<12, 3>(0, 0) = cube_F;

  // Compute gripper when lie on xz plane with center mount hole at origin
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.rotate(Eigen::AngleAxisd(-EIGEN_PI / 2, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d mountHole(meshInfo.maximum.x() + plateThickness,
                            minCoord.y() + m_mountOriginY + 0.031,
                            projectedCenterOfMass.x());
  m_mountHole = mountHole;
  mountHole = t * mountHole;
  t.pretranslate(-mountHole);
  gripper_raw_V.resize(gripper_V.rows(), 3);

  // Rotate back
  Eigen::Affine3d rotationInv = rotation.inverse();
  for (Eigen::Index i = 0; i < gripper_V.rows(); i++) {
    Eigen::Vector3d v = gripper_V.row(i);
    gripper_raw_V.row(i) = t * v;
    gripper_V.row(i) = rotationInv * v;
  }
}

Gripper::Gripper() {}

void Gripper::WriteDXF(const std::string& filename) const {
  DL_Dxf dxf;
  std::unique_ptr<DL_WriterA> dw(dxf.out(filename.c_str(), DL_VERSION_R12));
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

  double width = plateDimension.x() * 1000;
  double height = plateDimension.y() * 1000;

  // Drawing starts here

  // Plate
  dxf.writePolyline(*dw, DL_PolylineData(4, 0, 0, 1), defaultAttribute);
  dxf.writeVertex(*dw, DL_VertexData(0, 0));
  dxf.writeVertex(*dw, DL_VertexData(0, height));
  dxf.writeVertex(*dw, DL_VertexData(width, height));
  dxf.writeVertex(*dw, DL_VertexData(width, 0));
  dxf.writePolylineEnd(*dw);

  // Contact fitter
  for (ssize_t i = 0; i < rodLocations.rows(); i++) {
    dxf.writeCircle(*dw,
                    DL_CircleData(rodLocations.row(i).x() * 1000,
                                  rodLocations.row(i).y() * 1000,
                                  0,
                                  (m_rodRadius + m_rodClearance) * 1000),
                    defaultAttribute);

    for (int j = 0; j < 3; j++) {
      double angle = EIGEN_PI * 2 / 3 * j;
      Eigen::RowVector2d screwLocation =
          rodLocations.row(i) * 1000 +
          (m_fitterMountRadius * 1000) *
              Eigen::RowVector2d(cos(angle), sin(angle));
      dxf.writeCircle(*dw,
                      DL_CircleData(screwLocation.x(),
                                    screwLocation.y(),
                                    0,
                                    m_fitterScrewRadius * 1000),
                      defaultAttribute);
    }

    // Mount
    double center = m_cmLocationX * 1000;  // mm
    double originY = m_mountOriginY * 1000;
    dxf.writeCircle(
        *dw, DL_CircleData(center, originY + 31, 0, 5.8 / 2), defaultAttribute);
    dxf.writeArc(*dw,
                 DL_ArcData(center - 12.5, originY + 7, 0, 2.5, 0, 180),
                 defaultAttribute);
    dxf.writeArc(*dw,
                 DL_ArcData(center - 12.5, originY + 5, 0, 2.5, 180, 0),
                 defaultAttribute);
    dxf.writeLine(
        *dw,
        DL_LineData(center - 15, originY + 5, 0, center - 15, originY + 7, 0),
        defaultAttribute);
    dxf.writeLine(
        *dw,
        DL_LineData(center - 10, originY + 5, 0, center - 10, originY + 7, 0),
        defaultAttribute);

    dxf.writeArc(
        *dw,
        DL_ArcData(m_cmLocationX * 1000 + 11.5, originY + 6, 0, 2.5, 90, 270),
        defaultAttribute);
    dxf.writeArc(
        *dw,
        DL_ArcData(m_cmLocationX * 1000 + 13.5, originY + 6, 0, 2.5, 270, 90),
        defaultAttribute);
    dxf.writeLine(
        *dw,
        DL_LineData(
            center + 11.5, originY + 8.5, 0, center + 13.5, originY + 8.5, 0),
        defaultAttribute);
    dxf.writeLine(
        *dw,
        DL_LineData(
            center + 11.5, originY + 3.5, 0, center + 13.5, originY + 3.5, 0),
        defaultAttribute);
  }

  dw->sectionEnd();

  dxf.writeObjects(*dw);
  dxf.writeObjectsEnd(*dw);

  dw->dxfEOF();
  dw->close();
}

static std::string ToString(const Eigen::Vector3d& v) {
  std::stringstream ss;
  ss << std::setprecision(9) << v.x() << "," << v.y() << "," << v.z();
  return ss.str();
}

void Gripper::WriteRAPID(const std::string& filename) const {
  double maxLength = 0;
  for (double l : rodLengths)
    maxLength = std::max(maxLength, l);

  std::cout << "mountHole: " << m_mountHole << "\n";

  Eigen::Vector3d aquisition = m_mountHole;
  Eigen::Vector3d lineup =
      m_mountHole + Eigen::Vector3d(maxLength + 0.01, 0, 0);
  Eigen::Vector3d lift = m_mountHole + Eigen::Vector3d(0, 0.075, 0);

  Eigen::Matrix3d t;
  t.setZero();
  t(0, 0) = -1000;
  t(1, 2) = 1000;
  t(2, 1) = 1000;

  Eigen::Vector3d offset(498, -50, 96);

  Eigen::Vector3d r_aquisition = t * aquisition + offset;
  Eigen::Vector3d r_lineup = t * lineup + offset;
  Eigen::Vector3d r_lift = t * lift + offset;

  std::ofstream myfile(filename);
  myfile << "    CONST robtarget TargetLineup:=[[" << ToString(r_lineup)
         << "],[0,0.707106781,0,0.707106781],[-1,-1,-2,1],"
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
  myfile << "    CONST robtarget TargetAqusition:=[[" << ToString(r_aquisition)
         << "],[0,0.707106781,0,0.707106781],[-1,-1,-2,1],"
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
  myfile << "    CONST robtarget TargetLift:=[[" << ToString(r_lift)
         << "],[0,0.707106781,0,0.707106781],[-1,-1,-2,1],"
            "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
}

}  // namespace gripper
