#include "dbscan.h"

namespace wlby::depth_segmentation {

    int DBSCAN::run() {
        std::vector<DBSCANPoint>::iterator iter;
        for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
            if (iter->clusterID == UNCLASSIFIED) {
                if (expandCluster(*iter, clusterID) != FAILURE) {
                    clusterID += 1;
                }
            }
        }

        return 0;
    }

    int DBSCAN::expandCluster(DBSCANPoint point, int clusterID) {
        std::vector<int> clusterSeeds = calculateCluster(point);

        if (clusterSeeds.size() < m_minPoints) {
            point.clusterID = NOISE;
            return FAILURE;
        } else {
            int index = 0, indexCorePoint = 0;
            std::vector<int>::iterator iterSeeds;
            for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds) {
                m_points.at(*iterSeeds).clusterID = clusterID;
                if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y &&
                    m_points.at(*iterSeeds).z == point.z) {
                    indexCorePoint = index;
                }
                ++index;
            }
            clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

            for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i) {
                std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

                if (clusterNeighors.size() >= m_minPoints) {
                    std::vector<int>::iterator iterNeighors;
                    for (iterNeighors = clusterNeighors.begin();
                         iterNeighors != clusterNeighors.end(); ++iterNeighors) {
                        if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED ||
                            m_points.at(*iterNeighors).clusterID == NOISE) {
                            if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED) {
                                clusterSeeds.push_back(*iterNeighors);
                                n = clusterSeeds.size();
                            }
                            m_points.at(*iterNeighors).clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }

    std::vector<int> DBSCAN::calculateCluster(DBSCANPoint point) {
        int index = 0;
        std::vector<DBSCANPoint>::iterator iter;
        std::vector<int> clusterIndex;
        for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
            if (calculateDistance(point, *iter) <= m_epsilon) {
                clusterIndex.push_back(index);
            }
            index++;
        }
        return clusterIndex;
    }

    inline double DBSCAN::calculateDistance(const DBSCANPoint &pointCore, const DBSCANPoint &pointTarget) {
        return pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2) +
               pow(pointCore.z - pointTarget.z, 2);
    }

    bool DBSCAN::getMaxNumIdx(std::vector<int> &maxIdx) {
        std::unordered_map<int, std::pair<std::vector<int>, int>> map_store;
        for (int i = 0; i < m_points.size(); i++) {

            if (m_points[i].clusterID == -1) {
                continue;
            }
            map_store[m_points[i].clusterID].first.push_back(i);
            map_store[m_points[i].clusterID].second += m_points[i].PointNum;
        }
        int maxNum = -1;
        for (const auto &it: map_store) {
            if (it.second.second > maxNum) {
                maxNum = it.second.second;
                maxIdx = it.second.first;
            }
        }
        return maxNum <= 0 ? false : true;
    }

}
