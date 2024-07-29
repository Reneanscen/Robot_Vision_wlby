//
// Created by jazzey on 2023/11/15.
//

#ifndef YOLOV8_NCNN_VARIABLES_H
#define YOLOV8_NCNN_VARIABLES_H

#include <string>
#include <map>

namespace wlby::yolo{

    constexpr int Class_Nums = 9;

    static const std::string Class_names[Class_Nums] = {
     	"bed" , "bedside_table", "charging_station", 
     	"fold_table","refrigerator", "sink" ,
     	"table", "tv", "water_dispenser"
    };

    static const std::map<std::string, uint8_t> Classname_semanticlabel_map = {
            {"bed", 0}, {"bedside_table", 1}, {"charging_station", 2}, {"fold_table", 3},
            {"refrigerator", 4}, {"sink", 5}, {"table", 6},
            {"tv", 7}, {"water_dispenser", 8}
    };

   /*static const std::string Class_names[Class_Nums] = {
            "person", "bicycle", "car", "motorcycle", "airplane",
            "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird",
            "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard","tennis racket", "bottle",
            "wine glass", "cup", "fork","knife", "spoon",
            "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog","pizza", "donut",
            "cake", "chair", "couch","potted plant", "bed",
            "dining table", "toilet", "tv","laptop", "mouse",
            "remote", "keyboard", "cell phone","microwave", "oven",
            "toaster", "sink", "refrigerator", "book", "clock",
            "vase", "scissors", "teddy bear","hair drier", "toothbrush"
    };*/

   /*static const std::map<std::string, uint8_t> Classname_semanticlabel_map = {
           {"person", 0},         {"bicycle", 1},    {"car", 2},            {"motorcycle", 3},    {"airplane", 4},
           {"bus", 5},            {"train", 6},      {"truck", 7},          {"boat", 8},          {"traffic light", 9},
           {"fire hydrant", 10},  {"stop sign", 11}, {"parking meter", 12}, {"bench", 13},        {"bird", 14},
           {"cat", 15},           {"dog", 16},       {"horse", 17},         {"sheep", 18},        {"cow", 19},
           {"elephant", 20},      {"bear", 21},      {"zebra", 22},         {"giraffe", 23},      {"backpack", 24},
           {"umbrella", 25},      {"handbag", 26},   {"tie", 27},           {"suitcase", 28},     {"frisbee", 29},
           {"skis", 30},          {"snowboard", 31}, {"sports ball", 32},   {"kite", 33},         {"baseball bat", 34},
           {"baseball glove", 35},{"skateboard", 36},{"surfboard", 37},     {"tennis racket", 38},{"bottle", 39},
           {"wine glass", 40},    {"cup", 41},       {"fork", 42},          {"knife", 43},        {"spoon", 44},
           {"bowl", 45},          {"banana", 46},    {"apple", 47},         {"sandwich", 48},     {"orange", 49},
           {"broccoli", 50},      {"carrot", 51},    {"hot dog", 52},       {"pizza", 53},        {"donut", 54},
           {"cake", 55},          {"chair", 56},     {"couch", 57},         {"potted plant", 58}, {"bed", 59},
           {"dining table", 60},  {"toilet", 61},    {"tv", 62},            {"laptop", 63},       {"mouse", 64},
           {"remote", 65},        {"keyboard", 66},  {"cell phone", 67},    {"microwave", 68},    {"oven", 69},
           {"toaster", 70},       {"sink", 71},      {"refrigerator", 72},  {"book", 73},         {"clock", 74},
           {"vase", 75},          {"scissors", 76},  {"teddy bear", 77},    {"hair drier", 78},   {"toothbrush", 79},
           };*/
    
    /*static const std::string class_names[Class_Nums] = {
            "table", "tv_table", "chair", "sink", "water_dispenser",
            "door", "tv", "toilet", "bedside_table", "bed",
            "refrigerator", "cupboard", "hand washing sink"
    };*/

    static const unsigned char colors[80][3] = {
            {56,  0,   255},
            {226, 255, 0},
            {0,   94,  255},
            {0,   37,  255},
            {0,   255, 94},
            {255, 226, 0},
            {0,   18,  255},
            {255, 151, 0},
            {170, 0,   255},
            {0,   255, 56},
            {255, 0,   75},
            {0,   75,  255},
            {0,   255, 169},
            {255, 0,   207},
            {75,  255, 0},
            {207, 0,   255},
            {37,  0,   255},
            {0,   207, 255},
            {94,  0,   255},
            {0,   255, 113},
            {255, 18,  0},
            {255, 0,   56},
            {18,  0,   255},
            {0,   255, 226},
            {170, 255, 0},
            {255, 0,   245},
            {151, 255, 0},
            {132, 255, 0},
            {75,  0,   255},
            {151, 0,   255},
            {0,   151, 255},
            {132, 0,   255},
            {0,   255, 245},
            {255, 132, 0},
            {226, 0,   255},
            {255, 37,  0},
            {207, 255, 0},
            {0,   255, 207},
            {94,  255, 0},
            {0,   226, 255},
            {56,  255, 0},
            {255, 94,  0},
            {255, 113, 0},
            {0,   132, 255},
            {255, 0,   132},
            {255, 170, 0},
            {255, 0,   188},
            {113, 255, 0},
            {245, 0,   255},
            {113, 0,   255},
            {255, 188, 0},
            {0,   113, 255},
            {255, 0,   0},
            {0,   56,  255},
            {255, 0,   113},
            {0,   255, 188},
            {255, 0,   94},
            {255, 0,   18},
            {18,  255, 0},
            {0,   255, 132},
            {0,   188, 255},
            {0,   245, 255},
            {0,   169, 255},
            {37,  255, 0},
            {255, 0,   151},
            {188, 0,   255},
            {0,   255, 37},
            {0,   255, 0},
            {255, 0,   170},
            {255, 0,   37},
            {255, 75,  0},
            {0,   0,   255},
            {255, 207, 0},
            {255, 0,   226},
            {255, 245, 0},
            {188, 255, 0},
            {0,   255, 18},
            {0,   255, 75},
            {0,   255, 151},
            {255, 56,  0}
    };


}

#endif //YOLOV8_NCNN_VARIABLES_H
