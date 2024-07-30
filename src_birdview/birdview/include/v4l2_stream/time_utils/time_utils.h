//
// Created by xu on 24-2-5.
//

#ifndef BIRD_VISION_TIME_UTILS_H
#define BIRD_VISION_TIME_UTILS_H

#include "chrono"

// TicToc计时类
class TicToc{
public:
    TicToc(){
        Tic();
    }

    void Tic(){
        start_point_ = std::chrono::steady_clock::now();
    }

    double Toc(){
        end_point_ = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_point_ - start_point_).count();
        return duration;
    }

private:
    std::chrono::steady_clock::time_point start_point_, end_point_;
};


#endif //BIRD_VISION_TIME_UTILS_H
