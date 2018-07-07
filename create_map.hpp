#ifndef CREATE_MAP_HPP
#define CREATE_MAP_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>

class create_map{
public:
    create_map();
    void setup();
    void conversion(float *x,float *y);
    void set_rev_rad(float deg);
    float ret_rev_rad();
    void origin(float x,float y);
    float return_ox();
    float return_oy();
    int return_sx();
    int return_sy();

    void create(float dis,float rad,int step,int flag);
    void show();
    float return_x();
    float return_y();
    int return_flag();

    float judge();
    float return_val_x();
    float return_val_y();


    void hit_switch(int *hit);
    void med_create(float med,int hit);
    int ret_hit_flag();

private:
    cv::Mat img;
    cv::Mat copy;
    cv::Mat back;
    int ox;
    int oy;
    float sx;
    float sy;
    float rev_rad;
    float ret_x;
    float ret_y;
    float value_x;
    float value_y;

    int step_check[1440];
    int step;
    int ret_flag;
    int hit_flag;
};
#endif
