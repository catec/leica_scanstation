/**
 * @file ptx_2_pcd.cpp
 * @copyright Copyright (c) 2020, FADA-CATEC
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
 */

#include "leica_scanstation_utils/ptx_2_pcd.h"

void ptx_2_pcd::operator()(const std::string& ptx_fname)
{
  assert(extension(ptx_fname) == ".ptx");

  std::string pcd_fname = ptx_fname.substr(0, ptx_fname.find(".ptx")) + std::string(".pcd");

  std::ifstream ptx_file;
  std::ofstream pcd_file;

  ptx_file.open(ptx_fname);
  pcd_file.open(pcd_fname);
  std::string line;

  std::getline(ptx_file, line);
  _cols = atoi(line.c_str());

  std::getline(ptx_file, line);
  _rows = atoi(line.c_str());

  // Mat transf scaner
  std::getline(ptx_file, line);
  fillMatrixRow(_scan_tf, parseLine(line), 3);
  for (int i = 0; i < 3; i++)
  {
    std::getline(ptx_file, line);
    fillMatrixRow(_scan_tf, parseLine(line), i);
  }
  // Mat transf puntos
  for (int i = 0; i < 4; i++)
  {
    std::getline(ptx_file, line);
    fillMatrixRow(_point_tf, parseLine(line), i);
  }

  pcd_file << buildHeader();
  for (; std::getline(ptx_file, line);)
  {
    auto data = parseLine(line);
    pcd_file << processPointLine(data) << "\n";
  }
  ptx_file.close();
  pcd_file.close();
}

inline std::string ptx_2_pcd::extension(const std::string& fname) noexcept
{
  std::size_t it = fname.rfind(".");
  return it == std::string::npos ? "" : fname.substr(it);
}

inline std::vector<float> ptx_2_pcd::parseLine(const std::string& line)
{
  std::size_t pos = line.find(" ");
  std::size_t prev_pos = 0;
  std::vector<float> elements;
  while (pos != std::string::npos)
  {
    elements.emplace_back(atof(line.substr(prev_pos, pos).c_str()));
    prev_pos = pos + 1;
    pos = line.find(" ", prev_pos);
  }
  elements.emplace_back(atof(line.substr(prev_pos).c_str()));
  return elements;
}

inline std::string ptx_2_pcd::transformPoint(const Eigen::Vector4f& point)
{
  Eigen::Vector4f tf_point = point.transpose() * _point_tf;
  std::stringstream o_point;
  // [x, y, z, 1]
  for (int i = 0; i < 2; i++)
  {
    o_point << *(tf_point.data() + i) << " ";
  }
  o_point << *(tf_point.data() + 2);
  return o_point.str();
}

template <typename T, int Rows, int Cols>
inline void ptx_2_pcd::fillMatrixRow(Eigen::Matrix<T, Rows, Cols>& matrix, const std::vector<T>& data, int row)
{
  assert(matrix.cols() == data.size());
  for (int i = 0, size = data.size(); i < size; i++)
    matrix(row, i) = data[i];
}

inline std::string ptx_2_pcd::processPointLine(const std::vector<float>& line)
{
  auto end = std::next(line.begin(), 3);
  std::vector<float> v_point(line.begin(), end);
  v_point.emplace_back(1.0);
  Eigen::Vector4f point(v_point.data());

  return transformPoint(point);
}

inline std::string ptx_2_pcd::buildHeader()
{
  std::stringstream sstream;
  sstream << "# .PCD v.7 - Point Cloud Data file format\n";
  sstream << "VERSION .7\n";
  sstream << "FIELDS x y z\n";
  sstream << "SIZE 4 4 4\n";
  sstream << "TYPE F F F\n";
  sstream << "COUNT 1 1 1\n";
  sstream << "WIDTH " << _cols << "\n";
  sstream << "HEIGHT " << _rows << "\n";
  sstream << "VIEWPOINT " << _scan_tf.row(3) << " 1 0 0 0\n";
  sstream << "POINTS " << _cols * _rows << "\n";
  sstream << "DATA ascii\n";
  return sstream.str();
}