#include "tracker.h"


Tracker::Tracker() {
    id_ = 0;
}

float Tracker::CalculateIou(const msg_box::Box& det, const Track& track) {
    auto trk = track.GetStateAsBbox();
    // get min/max points
    auto detx = det.x;
    auto dety = det.y;
    auto detl = fabs(det.l);
    auto detw = abs(det.w);
    auto trackx = trk[0];
    auto tracky = trk[1];
    auto trackl = fabs(trk[2]);
    auto trackw = fabs(trk[3]);
    float score = 999;
    float distan = sqrt( pow((det.x-trackx) ,2) * pow((det.y-tracky) ,2))+0.1;
    float min_area = std::min(detl*detw ,trackl*trackw );
    float  area = fabs(detl*detw - trackl*trackw)/min_area+0.1;
    if(detl*detw - trackl*trackw<0.1)area=0.1;
    //截止阈值
    if(distan>10 )return 0.001;
    //if(area>3 )return 0.001;
    //超大目标默认剔除，不跟踪
    //if(detl>8 || detw>8|| trackl>8|| trackw >8)return 0.001;
    //return 30.0/(0.01+distan*area);
    return 30.0/(1.0+distan*distan);
}


void Tracker::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 1.0f;
            }
        }
    }

//    // Display begin matrix state.
//    for (size_t row = 0 ; row < nrows ; row++) {
//        for (size_t col = 0 ; col < ncols ; col++) {
//            std::cout.width(10);
//            std::cout << matrix(row,col) << ",";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;
    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

//    // Display solved matrix.
//    for (size_t row = 0 ; row < nrows ; row++) {
//        for (size_t col = 0 ; col < ncols ; col++) {
//            std::cout.width(2);
//            std::cout << matrix(row,col) << ",";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


void Tracker::AssociateDetectionsToTrackers(const std::vector<msg_box::Box>& detection,
                                            std::map<int, Track>& tracks,
                                            std::map<int, msg_box::Box>& matched,
                                            std::vector<msg_box::Box>& unmatched_det,
                                            float iou_threshold) {

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detection.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= 10) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}


void Tracker::Run(const std::vector<msg_box::Box> & detections) {

    /*** Predict internal tracks from previous frame ***/
    for (auto &track : tracks_) {
        track.second.dt_ = this->dt;
        track.second.Predict();
        
    }

    // Hash-map between track ID and associated detection bounding box
    std::map<int, msg_box::Box> matched;
    // vector of unassociated detections
    std::vector<msg_box::Box> unmatched_det;

    // return values - matched, unmatched_det
    if (!detections.empty()) {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }

    /*** Update tracks with associated bbox ***/
    for (const auto &match : matched) {
        const auto &ID = match.first;
        tracks_[ID].track_id = match.second.sensor_id;
        tracks_[ID].Update(match.second);
        tracks_[ID].JudgeMove(match.second );
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det) {
        
        Track tracker;
        tracker.Init(det);
        // Create new track and generate new ID
        tracks_[id_++] = tracker;
    }

    /*** Delete lose tracked tracks ***/
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        //if (it->second.coast_cycles_ > kMaxCoastCycles) {
            //coast_cycles_  在predict时++，在update时清零，调用update即匹配上了
            //该参数表示一个未匹配上目标的预测次数，故此处决定一个目标没匹配上时预测几次再erase
            //这样当在预测未删除时间内，若目标又出现则会被继续跟踪
            //if (it->second.coast_cycles_ > 0)表示没匹配上就直接delete
            if (it->second.coast_cycles_ > 10) {
            it = tracks_.erase(it);
        } else {
            it++;
        }
    }
}


std::map<int, Track> Tracker::GetTracks() {
    return tracks_;
}