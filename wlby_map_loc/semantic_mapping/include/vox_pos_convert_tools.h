//
// Created by jazzey on 2023/11/21.
//

#ifndef SEMANTIC_MAPPING_VOXPOSCONVERTTOOLS_H
#define SEMANTIC_MAPPING_VOXPOSCONVERTTOOLS_H

#include "PoseUtils/Pose3DDataStruct.h"
#include <cmath>

namespace wlby::semantic_integrator {
    class VoxPosConvertTools {

    public:
        VoxPosConvertTools(const Localization::Pos3D &origin, double resolution, const Localization::Pos3D &map_size) :
            resolution_(resolution), origin_(origin), map_size_(map_size) {

            //根据预设的map_size大小进行栅格数量的计算
            grid_size_._ix = ceil(map_size._xyz[0] / resolution_);
            grid_size_._iy = ceil(map_size._xyz[1] / resolution_);
            grid_size_._iz = ceil(map_size._xyz[2] / resolution_);
            grid_size_yz_ = grid_size_._iy * grid_size_._iz;

        }

        void pos2Vox(const Localization::Pos3D &pos, Localization::Pos3Di &vox) {
            vox._ix = floor((pos._xyz[0] - origin_._xyz[0]) / resolution_);
            vox._iy = floor((pos._xyz[1] - origin_._xyz[1]) / resolution_);
            vox._iz = floor((pos._xyz[2] - origin_._xyz[2]) / resolution_);
        }

        void vox2Pos(const Localization::Pos3Di &vox, Localization::Pos3D &pos) {
            pos._xyz[0] = (vox._ix + 0.5) * resolution_ + origin_._xyz[0];
            pos._xyz[1] = (vox._iy + 0.5) * resolution_ + origin_._xyz[1];
            pos._xyz[2] = (vox._iz + 0.5) * resolution_ + origin_._xyz[2];
        }

        bool voxInRange(const Localization::Pos3Di &vox) const
        {
            return (vox._ix >= 0 && vox._ix < grid_size_._ix && vox._iy >= 0 && vox._iy < grid_size_._iy && vox._iz >= 0 && vox._iz < grid_size_._iz);
        }

        uint64 vox2Idx(const Localization::Pos3Di &vox) const
        {
            return vox._ix * grid_size_yz_ + vox._iy * grid_size_._iz + vox._iz;
        }

        Localization::Pos3Di idx2Vox(uint64 idx) const
        {
            Localization::Pos3Di pos(idx / grid_size_yz_,
                                     idx % (grid_size_yz_) / grid_size_._iz,
                                     idx % grid_size_._iz);
            return pos;
        }




    public:

        double resolution_ = 0.025;
        Localization::Pos3D origin_;
        Localization::Pos3D map_size_;

        Localization::Pos3Di grid_size_;
        uint64 grid_size_yz_;


    };
}

#endif //SEMANTIC_MAPPING_VOXPOSCONVERTTOOLS_H
