#include "track.h"
// #include "msg_box/Box.h"

Track::Track() : kf_(8, 4) {

    /*** Define constant velocity model ***/
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    kf_.F_ <<
           1, 0, 0, 0, dt_, 0, 0, 0,
            0, 1, 0,0, 0, dt_, 0, 0,
            0, 0, 1, 0, 0, 0, dt_, 0,
            0, 0, 0, 1, 0, 0, 0, dt_,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1;
// kf_.F_ <<
//            1, 0, 1, 0, 
//             0, 1, 0, 1,
//             0, 0, 1, 0, 
//             0, 0, 0, 1;
            
    // Give high uncertainty to the unobservable initial velocities
    kf_.P_ <<
           100, 0, 0, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0, 0, 0,
            0, 0, 0, 100, 0, 0, 0, 0,
            0, 0, 0, 0, 100, 0, 0, 0,
            0, 0, 0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 0, 0, 100;
    //  kf_.P_ <<
    //        10, 0, 0, 0, 
    //         0, 10, 0, 0, 
    //         0, 0, 10000, 0, 
    //         0, 0, 0, 10000;

    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0;
        //     kf_.H_ <<
        //    1, 0, 0, 0,
        //     0, 1, 0, 0;

    kf_.Q_ <<
           10, 0, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 0, 100, 0, 0, 0,
            0, 0, 0, 0, 0,100, 0, 0,
            0, 0, 0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 0, 0, 100;
// kf_.Q_ <<
//            1, 0, 0, 0,
//             0, 1, 0, 0, 
//             0, 0, 0.1, 0, 
//             0, 0, 0, 0.1;
    kf_.R_ <<
           1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;
}


// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Track::Predict() {
    //std::cout<<"predict"<<dt_<<std::endl;
    kf_.F_ <<
           1, 0, 0, 0, dt_, 0, 0, 0,
            0, 1, 0,0, 0, dt_, 0, 0,
            0, 0, 1, 0, 0, 0, dt_, 0,
            0, 0, 0, 1, 0, 0, 0, dt_,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1;
    kf_.Predict();
    
    // hit streak count will be reset
    if (coast_cycles_ > 0) {
        //coast_cycles_  在predict时++，在update时清零，调用update即匹配上了
            //该参数表示一个未匹配上目标的预测次数，当coast_cycles_大于0证明这个目标被跟丢，故hit_streak_清零
        //hit_streak_在初始化或update时++，表示一个目标被连续跟踪的次数
        hit_streak_ = 0;
    }
    // accumulate coast cycle count
    coast_cycles_++;
}


// Update matched trackers with assigned detections
void Track::Update(const msg_box::Box& bbox) {

    // get measurement update, reset coast cycle count
    coast_cycles_ = 0;
    // accumulate hit streak count
    hit_streak_++;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    kf_.Update(observation);
}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Track::Init(const msg_box::Box &bbox) {
    //std::cout<<"init first"<<std::endl;
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    hit_streak_++;
    
}

void Track::JudgeMove(const msg_box::Box& bbox)
{
    tracking_time  +=  dt_;
    tracking_time=tracking_time>999?00:tracking_time;
    if(q_dt.size() != q_s.size())
    {
        std::cout<<"error --------------------------------------------------JudgeMove"<<std::endl;
    }else
    {
        double sum_dt =0;
        for(int i = 1 ; i < q_dt.size() ; i++)
        {
            sum_dt += q_dt.at(i);
        }
        if(q_dt.size()>20 || sum_dt>2)
        {
            q_dt.pop_front();
            q_s.pop_front();
            q_dt.push_back(dt_);
            q_s.push_back(std::make_pair(bbox.x,bbox.y));
        }
        else
        {
            q_dt.push_back(dt_);
            q_s.push_back(std::make_pair(bbox.x,bbox.y));
        }
    }
       
    if(tracking_time>1  )
    {
        //限制1s内最大位移
        //限制1s内最大速度
        double maxsx = q_s.front().first, minsx = q_s.front().first ;
        double maxsy = q_s.front().second, minsy = q_s.front().second ;
        std::vector<double> v_vx , v_vy;
        for(int i = 0 ; i < q_s.size() ; i++)
        {
            if(maxsx<q_s.at(i ).first)maxsx = q_s.at(i ).first;
            if(minsx>q_s.at(i ).first)minsx = q_s.at(i ).first;
            if(maxsy<q_s.at(i ).second)maxsy = q_s.at(i ).second;
            if(minsy>q_s.at(i ).second)minsy = q_s.at(i ).second;
            if(i>0)
            {
                v_vx.push_back( (q_s.at(i ).first - q_s.at(i -1).first)/q_dt.at(i) );
                v_vy.push_back( (q_s.at(i ).second - q_s.at(i -1).second)/q_dt.at(i) );
            }else{
                v_vx.push_back( 0 );
                v_vy.push_back( 0 );
            }
            
        } 
        //限制1s内最大速度
        double maxvx = v_vx.front(), minvx = v_vx.front() ;
        double maxvy = v_vy.front(), minvy = v_vy.front() ;
         for(int i = 0 ; i < v_vy.size() ; i++)
        {
            if(maxvx<v_vx.at(i ))maxvx = v_vx.at(i );
            if(minvx>v_vx.at(i ))minvx = v_vx.at(i );
            if(maxvy<v_vy.at(i ))maxvy = v_vy.at(i );
            if(minvy>v_vy.at(i ))minvy = v_vy.at(i );
        }
        if(maxsx - minsx  >= 1 || maxsy - minsy  >= 1)
        {
            if(maxsx - minsx  <20 && maxsy - minsy  <20 && fabs(maxvx)<20 &&
             fabs(maxvy)<20 && fabs(minvy)<20 && fabs(minvx)<20&&fabs(maxvy)>0)
            {
                if(fabs(maxvx-minvx)<5 && fabs(maxvy-minvy)<5)move_flag = 1;
            }
        }

    }


    if(tracking_time<=0 || q_s.size() == 0 || q_dt.size() == 0 )
        {
            return;
        }
}
/**
 * Returns the current bounding box estimate
 * @return
 */
Eigen::VectorXd Track::GetStateAsBbox() const {
    return ConvertStateToBbox(kf_.x_);
}


float Track::GetNIS() const {
    return kf_.NIS_;
}


/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Track::ConvertBboxToObservation(const msg_box::Box& bbox) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
    auto l = static_cast<float>(bbox.l);
    auto w = static_cast<float>(bbox.w);
    float center_x = bbox.x ;
    float center_y = bbox.y ;
    observation << center_x, center_y, l, w;
    return observation;
}


/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
Eigen::VectorXd Track::ConvertStateToBbox(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(8);
    auto center_x = static_cast<double>(state[0] );
    auto center_y = static_cast<double>(state[1] );
    auto width = static_cast<double>(state[2]);
    auto height = static_cast<double>(state[3]);
    auto center_vx = static_cast<double>(state[4] );
    auto center_vy = static_cast<double>(state[5] );

    observation << center_x, center_y, width, height,center_vx,center_vy,tracking_time,move_flag;
    return observation;
}