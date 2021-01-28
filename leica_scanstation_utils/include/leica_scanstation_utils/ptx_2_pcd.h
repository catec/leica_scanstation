/**
 * @file ptx_2_pcd.h
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


#pragma once
#ifndef __PTX_2_PCD_H__
#define __PTX_2_PCD_H__

#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include <iostream>

#endif

class ptx_2_pcd
{
public:
    /**
     * @brief Construct a new ptx 2 pcd object
     *
     */
    ptx_2_pcd(){};

    /**
     * @brief Destroy the ptx 2 pcd object
     *
     */
    ~ptx_2_pcd(){};

    /**
     * @brief Perform conversion
     *
     */
    void operator()(const std::string&);

private:
    /** @brief   Number of matrix rows  */
    int rows_{};

    /** @brief   Number of matrix cols  */
    int cols_{};

    /** @brief   Cloud's points transformation matrix */
    Eigen::Matrix4f point_tf_{};

    /** @brief   Scanner's POV transformation matrix  */
    Eigen::Matrix<float, 4, 3> scan_tf_;

    /**
     * @brief  Parse and extract file extension
     *
     * @return std::string Filename's extension
     */
    std::string extension(const std::string&) noexcept;

    /**
     * @brief  Performs conversion from ASCII to float until it finds EOL character
     *
     * @return std::vector<float> Converted line
     */
    std::vector<float> parseLine(const std::string&);

    /**
     * @brief Extracts the point from a line, applying a transformation in the processes
     *
     * @return std::string 3D processed point
     */
    std::string processPointLine(const std::vector<float>&);

    /**
     * @brief Applies scanner transformation matrix to a point
     *
     * @return std::string 3D transformed point
     */
    std::string transformPoint(const Eigen::Vector4f&);

    /**
     * @brief Fills in place an Eigen Matrix row with data from a std::vector
     *
     * @tparam T Fundamental type of matrix's data
     * @tparam Rows Number of rows
     * @tparam Cols Number of columns
     */
    template <typename T, int Rows, int Cols>
    void fillMatrixRow(Eigen::Matrix<T, Rows, Cols>&, const std::vector<T>&, int);

    /**
     * @brief pcd file header builder
     *
     * @return std::string File header
     */
    std::string buildHeader();
};
