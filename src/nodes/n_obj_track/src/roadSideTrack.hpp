#include <vector>
#include <map>
#include "msg_lidar_obj/msg_lidar_obj.h"
static const int MAX_PIX_Y = 1600;  // 整体范围
static const int MAX_PIX_X = 1600;  
static const int distance = 50;  // 两个点相差范围小于阈值，则为同一个点
std::map<int,std::string> id2name = {{0,"person"},{1,"bicycle"},{2,"car"},{3,"motorbike"},{4,"bus"},{5,"truck"},{6,"rider"},{7,"others"}};

//msg_lidar_obj::msg_lidar_obj
enum Status  // 状态
    {
        init,
        tracked,
        missing,
        abandon
    };

struct track_data
{
    int id;      // id
    int type;    // 类型
    int x;       // x 轴坐标
    int y;       // y 轴坐标
    int w;       // x 轴坐标
    int h;       // y 轴坐标
    int count;   // 连续出现次数
    Status status;
    std::vector<float> direction = std::vector<float>(2);           // 方向向量: x 轴方向的速度，y轴方向的速度.
    std::vector<std::vector<double>> trail;  // 轨迹
    long double timestamp;
};

class roadSideTrack
{
public:
    roadSideTrack(int num_class);
    ~roadSideTrack();
    void push(std::vector<msg_lidar_obj::msg_lidar_obj> &new_data, long double time);
    std::vector<std::map<int, track_data>> v_track;

private:
    void update(std::map<int, track_data>& m_tracked, std::vector<msg_lidar_obj::msg_lidar_obj> &new_data);
    int find(std::vector<msg_lidar_obj::msg_lidar_obj> &new_data, const int& id);
    std::vector<float> calculate_v(track_data& data);
    
    std::vector<std::vector<msg_lidar_obj::msg_lidar_obj>> v_newData;
    std::vector<std::map<int, track_data>> v_missing;
    long double cur_time;
    int num_class_;
};

void print_status(const std::vector<std::map<int, track_data>> &v_track)
{
    for (int i = 0; i < v_track.size(); ++i)
    {
        for (auto it = v_track[i].begin(); it != v_track[i].end(); ++it)
        {
            std::cout << "track id: " << it->first << " class: " << id2name[it->second.type] << " point: " << it->second.x
                      << " " << it->second.y << " status: " << it->second.status << " count: " << it->second.count
                      << " v_x: " << it->second.direction[0] << " v_y: " << it->second.direction[1] << " trail: ";
            
            std::cout << std::endl;
        }
    }
}

roadSideTrack::roadSideTrack(const int num_class):num_class_(num_class)
{
    for (int i = 0; i < num_class_; ++i)
    {
        v_track.push_back(std::map<int, track_data>());
        v_newData.push_back(std::vector<msg_lidar_obj::msg_lidar_obj>());
        v_missing.push_back(std::map<int, track_data>());
    }
}

void
roadSideTrack::push(std::vector<msg_lidar_obj::msg_lidar_obj> &new_data, long double time)
{
    cur_time = time;
    for (auto data : new_data)
    {
        v_newData[data.type].push_back(data);
    }
    for (int i = 0; i < v_newData.size(); ++i)
    {
        update(v_track[i], v_newData[i]);
        v_newData[i].clear();
    }
    print_status(v_track);
    std::cout << "******************************************************" << std::endl;
    print_status(v_missing);
}

void roadSideTrack::update(std::map<int, track_data>& m_tracked, std::vector<msg_lidar_obj::msg_lidar_obj> &new_data)
{
    for (auto it = m_tracked.begin(); it != m_tracked.end();)
    {
        //std::cout << "tracked id: " << it->first << std::endl;
        int index = find(new_data, it->first);
        if (index != -1)
        {
            // 匹配成功
            //std::cout << "匹配成功" << std::endl;
            if(++it->second.count >= 15)
            {
                it->second.status = Status::tracked;
            }
            else
            {
                it->second.status = Status::init;
            }
            //std::cout << "cur time: " << cur_time << " pre time: " << it->second.timestamp << std::endl;
            //std::cout << "time: " << cur_time - it->second.timestamp << std::endl;
            it->second.trail.push_back(std::vector<double>{new_data[index].x, new_data[index].y, new_data[index].w,
                                                           new_data[index].h, cur_time - it->second.timestamp});
            it->second.timestamp = cur_time;
            it->second.x = new_data[index].x;
            it->second.y = new_data[index].y;
            it->second.w = new_data[index].w;
            it->second.h = new_data[index].h;
            // 速度计算
            it->second.direction = calculate_v(it->second);
            new_data.erase(new_data.begin() + index);
            ++it;
        }
        else
        {
            // 消失
            if (it->second.status == Status::init)
            {
                // 未初始化成功，丢掉
                v_missing[it->second.type].erase(it->first);
                m_tracked.erase(it++);
                //std::cout << "未初始化成功，丢掉 success" << std::endl;
            }
            else
            {
                it->second.x += it->second.direction[0] * (cur_time - it->second.timestamp);
                it->second.y += it->second.direction[1] * (cur_time - it->second.timestamp);
                if (it->second.x >= MAX_PIX_X || it->second.x <= 0 || it->second.y >= MAX_PIX_Y || it->second.y <= 0)
                {
                    // 超出范围，丢掉
                    //std::cout << it->first<< " " << it->second.type << std::endl;
                    v_missing[it->second.type].erase(it->first);
                    m_tracked.erase(it++);
                    //std::cout << "超出范围，丢掉 success!" << std::endl;
                }
                else{
                    it->second.status = missing;
                    v_missing[it->second.type][it->first] = it->second;
                    it++;
                    //std::cout << "消失状态转换 success!" << std::endl;
                }
            }
        }
    }

    // 新增
    for (size_t i = 0; i < new_data.size(); ++i)
    {
        if (v_missing[new_data[i].type].count(new_data[i].tracker_id) > 0)
        {
            // 重新找回
            //std::cout << "重新找回 2" << std::endl;
            v_track[new_data[i].type][new_data[i].tracker_id].status = tracked;
            v_missing[new_data[i].type].erase(new_data[i].tracker_id);
            v_track[new_data[i].type][new_data[i].tracker_id].trail.push_back(std::vector<double>{new_data[i].x, new_data[i].y, new_data[i].w, new_data[i].h, cur_time - v_track[new_data[i].type][new_data[i].tracker_id].timestamp});
            v_track[new_data[i].type][new_data[i].tracker_id].timestamp = cur_time;
            
            v_track[new_data[i].type][new_data[i].tracker_id].count++;
            v_track[new_data[i].type][new_data[i].tracker_id].x = new_data[i].x;
            v_track[new_data[i].type][new_data[i].tracker_id].y = new_data[i].y;
            v_track[new_data[i].type][new_data[i].tracker_id].w = new_data[i].w;
            v_track[new_data[i].type][new_data[i].tracker_id].h = new_data[i].h;
            v_track[new_data[i].type][new_data[i].tracker_id].direction = calculate_v(v_track[new_data[i].type][new_data[i].tracker_id]);
            new_data.erase(new_data.begin() + i);
            --i;
            continue;
        }

        float min_dist = 1600;
        int index = -1;
        for (auto missing : v_missing[new_data[i].type])
        {
            float dist = sqrt(pow(missing.second.x - new_data[i].x, 2) + pow(missing.second.y - new_data[i].y, 2));
            
            if (dist < min_dist)
            {
                min_dist = dist;
                index = missing.first;
            }
        }
        if (min_dist < distance)
        {
            // 找回
            //std::cout << "找回 track id: " << index << "new data id: " << new_data[i].tracker_id << std::endl;

            v_track[new_data[i].type][index].status = tracked;
            
            v_track[new_data[i].type][index].trail.push_back(std::vector<double>{new_data[i].x, new_data[i].y, new_data[i].w, new_data[i].h, cur_time -
                                                                            v_track[new_data[i].type][index].timestamp});
            v_track[new_data[i].type][index].timestamp = cur_time;
            
            v_track[new_data[i].type][index].count++;
            v_track[new_data[i].type][index].x = new_data[i].x;
            v_track[new_data[i].type][index].y = new_data[i].y;
            v_track[new_data[i].type][index].w = new_data[i].w;
            v_track[new_data[i].type][index].h = new_data[i].h;
            v_track[new_data[i].type][index].direction = calculate_v(v_track[new_data[i].type][index]);
            v_missing[new_data[i].type].erase(index);
            new_data.erase(new_data.begin() + i);
            i--;
        }
    }

    for (auto newData: new_data)
    {
        // 添加新目标
        //std::cout << "添加新目标: " << newData.tracker_id << std::endl;
        track_data data;
        data.x = newData.x;
        data.y = newData.y;
        data.w = newData.w;
        data.h = newData.h;
        data.id = newData.tracker_id;
        data.count = 1;
        data.status = init;
        data.type = newData.type;
        data.direction = std::vector<float>{0.f, 0.f};
        data.trail.push_back(std::vector<double>{newData.x, newData.y, newData.w, newData.h, 0.0});
        data.timestamp = cur_time;
        v_track[data.type][data.id] = data;
    }
}

int roadSideTrack::find(std::vector<msg_lidar_obj::msg_lidar_obj> &new_data, const int& id)
{
    for (int i = 0; i < new_data.size(); ++i)
    {
        if (new_data[i].tracker_id == id)
            return i;
    }
    return -1;
}

std::vector<float> roadSideTrack::calculate_v(track_data& data)
{
    float time_sum = 0.f;
    float v_x = 0.f;
    float v_y = 0.f;
    std::vector<std::vector<double>> trail{data.trail.back()};
    
    std::cout << std::endl;
    for (int i = data.trail.size() - 2; i >= 0; --i)
    {
        time_sum += data.trail[i + 1][4];
        trail.push_back(data.trail[i]);
        
        if (time_sum > 1.f)
            break;
    }

    //float alpha = (1 - 2) / (1 - pow(2, trail.size() - 1));
    for (int i = 0; i < trail.size() - 1; ++i)
    {
        v_x += (trail[i][0] - trail[i + 1][0]) / trail[i][4];
        //    *alpha *pow(2, i);
        v_y += (trail[i][1] - trail[i + 1][1]) / trail[i][4];
    //    *alpha *pow(2, i);
    }
    return std::vector<float>{v_x / (trail.size() - 1), v_y / (trail.size() - 1)};
}
