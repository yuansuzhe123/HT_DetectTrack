#pragma once
#include "kalman_filter.h"
#include "msg_box/Box.h"
#include <deque>
class Track {
public:
    // Constructor
    Track();

    // Destructor
    ~Track() = default;

    void Init(const msg_box::Box& bbox);
    void Predict();
    void Update(const msg_box::Box& bbox);
    Eigen::VectorXd GetStateAsBbox() const;
    float GetNIS() const;

    int coast_cycles_ = 0, hit_streak_ = 0;
    int  track_id=-1;
    std::deque<double> q_dt;
    std::deque<std::pair<double , double>> q_s;
    double dt_;
    double tracking_time;
    int move_flag = 0;
    void JudgeMove(const msg_box::Box& bbox);
private:
    Eigen::VectorXd ConvertBboxToObservation(const msg_box::Box& bbox) const;
    Eigen::VectorXd ConvertStateToBbox(const Eigen::VectorXd &state) const;
    KalmanFilter kf_;
    
};
