#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <unordered_map>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

namespace wlby::depth_segmentation{
    struct DBSCANPoint
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
        int PointNum;
    };

    class DBSCAN {
    public:
        DBSCAN(unsigned int minPts, float eps, const std::vector<DBSCANPoint>& points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
        }
        ~DBSCAN(){}

        int run();
        std::vector<int> calculateCluster(DBSCANPoint point);
        int expandCluster(DBSCANPoint point, int clusterID);
        inline double calculateDistance(const DBSCANPoint& pointCore, const DBSCANPoint& pointTarget);

        int getTotalPointSize() {return m_pointSize;}
        int getMinimumClusterSize() {return m_minPoints;}
        int getEpsilonSize() {return m_epsilon;}
        int getClusterSize() {return clusterID;}

        bool getMaxNumIdx(std::vector<int>& maxIdx);


    public:
        std::vector<DBSCANPoint> m_points;

    private:
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
        int clusterID = 0;
    };
}


#endif // DBSCAN_H
