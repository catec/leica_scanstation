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
    void operator()(const std::string &);

private:

    /** @brief   number of matrix rows  */
    int _rows{};
    
    /** @brief   number of matrix cols  */
    int _cols{};

    /** @brief     */
    Eigen::Matrix4f _point_tf{};

    /** @brief     */
    Eigen::Matrix<float, 4, 3> _scan_tf;

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string extension(const std::string &) noexcept;

    /**
     * @brief 
     * 
     * @return std::vector<float> 
     */
    std::vector<float> parseLine(const std::string &);

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string processPointLine(const std::vector<float> &);

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string transformPoint(const Eigen::Vector4f &);

    /**
     * @brief 
     * 
     * @tparam T 
     * @tparam Rows 
     * @tparam Cols 
     */
    template <typename T, int Rows, int Cols>
    void fillMatrixRow(Eigen::Matrix<T, Rows, Cols> &, const std::vector<T> &, int);

    /**
     * @brief 
     * 
     * @return std::string 
     */
    std::string buildHeader();
};