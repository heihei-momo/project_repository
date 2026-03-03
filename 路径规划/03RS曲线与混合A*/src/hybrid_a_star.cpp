#include "hybrid_a_star/hybrid_a_star.h"
#include "hybrid_a_star/display_tools.h"
#include "hybrid_a_star/timer.h"
#include "hybrid_a_star/trajectory_optimizer.h"

#include <iostream>
// 参数分别为：steering_angle: 最大转向角度，转化为弧度使用。
// steering_angle_discrete_num: 转向角度的离散数量，控制转向角度的分辨率。
// segment_length: 从一个节点到另一个节点的路径段长度。
// segment_length_discrete_num: 路径段长度的离散数量，影响路径的平滑度和搜索的精度。
// wheel_base: 车辆的轴距，用于计算车辆转弯时的动态约束。
// steering_penalty, reversing_penalty, steering_change_penalty: 分别对应转向、倒车和转向变化的惩罚因子，用于在成本计算中对这些行为进行惩罚，以生成更符合实际驾驶情况的路径。
// shot_distance: 射线距离，用于在距离目标状态一定范围内时，采用解析解法（Reeds-Shepp路径）直接连接起点和终点。

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {//构造函数，初始化各种参数
    wheel_base_ = wheel_base; // 车辆轴距（前后轮距离）
    segment_length_ = segment_length; // 车辆轴距（前后轮距离）
    steering_radian_ = steering_angle * M_PI / 180.0; // 最大转向角度（弧度）
    steering_discrete_num_ = steering_angle_discrete_num;// 转向角度离散数量
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_; // 转向角度步长
    move_step_size_ = segment_length / segment_length_discrete_num; // 移动步长
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);// 离散化段数
    steering_penalty_ = steering_penalty;// 转向惩罚因子
    steering_change_penalty_ = steering_change_penalty;// 转向变化惩罚因子
    reversing_penalty_ = reversing_penalty; // 倒车惩罚因子
    shot_distance_ = shot_distance;// 触发解析解的距离阈值

    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;
        //Google Test / glog 常见的断言宏 CHECK_EQ，意思是检查两个值是否相等，如果不相等就报错并打印后面的提示信息。  

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));//通常是 Reeds-Shepp 路径 的类，用于计算车辆在给定起点和终点之间的可行路径
    //创建了一个 RSPath 对象（最小转弯半径作为参数）
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();//析构函数，清除内存
}

void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {//主要是计算车辆轮廓，且释放/分配状态网格和地图网格的内存
    SetVehicleShape(4.7, 2.0, 1.3);//计算车辆轮廓，并用点围起来
    
    map_x_lower_ = x_lower;//地图x最小值
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;//状态网格分辨率
    MAP_GRID_RESOLUTION_ = map_grid_resolution;//地图网格分辨率

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);//计算状态网格尺寸，几个格子

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);//计算地图网格尺寸，几个格子

    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;//释放地图数据
    }

    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];//存储地图网格尺寸

    if (state_node_map_) {//释放状态节点地图（复杂的三维数组）
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];//第一维，存二级指针
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {//重新分配状态节点地图
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {//布雷森汉姆直线算法，检查从点 (x0, y0) 到点 (x1, y1) 的线段是否：（网格坐标）
    // 碰撞障碍物 叉
    // 超出地图边界 叉
    // 完全畅通 勾
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }
    //如果线段更接近垂直（陡峭），交换x和y坐标，确保始终沿x轴方向进行遍历
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    //保证 x0 <= x1，确保从左向右遍历
    

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;//计算斜率，一定是正的
    decltype(delta_x) error = 0;//decltype(delta_x): 获取 delta_x 的类型，error是这个获取到的类型，累积误差值，用于决定何时调整y坐标
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);//N是x轴坐标差
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {//对于陡峭曲线（交换后）
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))//检查该向量代表的点，在地图上是否越界和该点是否有障碍物（沿着x轴以1.0为步长移动 )（网格坐标）
                || BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_,
                                        (x0 + i) * MAP_GRID_RESOLUTION_))//判断给定的点有没有超过地图边界（物理坐标）
                    ) {
                return false;
                //提供了双重保护
            }
        } else {//对于非陡峭曲线
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_,
                                        yk * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        }

        error += delta_error;//误差必须按固定方向连续累积，所以得按照一个方向遍历
        if (error >= 0.5) {
            yk += y_step;//yk通过累积误差进行调整
            error = error - 1.0;//y坐标调整了1个网格单位，所以误差需要相应减少1.0
        }
    }

    return true;
}

bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {//检查车辆位置情况下，四个边是否与障碍物碰撞
    Timer timer;//通常Timer的构造函数里会记录一个开始时间
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);//创建旋转矩阵

    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }//将车辆局部坐标系下的四个顶点全都变换到世界坐标系

    //将绝对坐标转换为珊格坐标
    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)// 顶点0 (x,y)
    );
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );
    //右下，右上，左上，左下

    double y1, y0, x1, x0;
    // 右上 → 右下（竖边），检查右边界上是否有障碍物
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // 左上 → 右上（上边），检查上边界是否有障碍位
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    //左下 → 左上（竖边），检查左边界是否有障碍物
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // 右下 → 左下（下边），检查下边界是否有障碍物
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();//timer.End() → 计算本次 CheckCollision 调用的耗时
    num_check_collision++;
    //两着配合可以计算平均耗时
    return true;
}

//map_data_ 是一维数组存储二维网格数据
bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {//检查该珊格坐标点在地图上是否越界和该点是否有障碍物
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_//网格不越界
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1)/*检查坐标在地图网格是否有障碍物*/);
}

bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {//检查该向量代表的点，在地图上是否越界和该点是否有障碍物
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];
    //提取向量的x，y分量

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {//用网格坐标设置障碍物
    if (x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
        || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {//检查障碍物所在的地图格子索引是否超出边界，超出边界了就不做处理
        return;
    }

    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;//用一维索引，表示障碍物占据了这个格子
}

void HybridAStar::SetObstacle(const double pt_x, const double pt_y) {//世界坐标 (pt_x, pt_y) 设置障碍物
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}
//根据参数类型来区分使用哪个函数

void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {//以后轴中心为原点(0,0)，rear_axle_dist是后轴到车尾的距离
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);//右下角
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);//右上角
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);//左上角
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);//左下角
    //四个点，定义车辆矩形轮廓

    //生成离散化，边界点
    const double step_size = move_step_size_;//移动步长
    const auto N_length = static_cast<unsigned int>(length / step_size);//长度方向点数
    const auto N_width = static_cast<unsigned int> (width / step_size);//宽度方向点数
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);//总点数

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
                                    //计算车辆边缘的单位方向向量，并归一化
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized/*乘法结果保证移动方向始终沿着车辆边缘*/ * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }//插点，形成长边

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();//朝左
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}//存储顺序：右边，左边，上边，下边

__attribute__((unused)) Vec2d HybridAStar::CoordinateRounding(const Vec2d &pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt)/*把连续的世界坐标(x,y)映射到离散的栅格地图索引(i,j)，返回珊格坐标*/);
}
//__attribute__((unused))告诉编译器这个函数 可能暂时未被使用，不要警告

Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {//栅格索引转回世界坐标系下格子中心点
    Vec2d pt; 
    //加0.5后，这样得到的世界坐标就是格子正中心点
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

Vec3i HybridAStar::State2Index(const Vec3d &state) const {//将坐标和角度离散化 
    Vec3i index;

    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_)/*将世界坐标x转为离散格子索引*/, 0), STATE_GRID_SIZE_X_ - 1/*保证索引不超过数组边界*/);//x方向离散化
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);//y方向离散化
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);//角度离散化
    //state[2] → 连续角度 θ，范围 [-π, π]
    return index;
}

Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d &pt) const {//把连续的世界坐标 (x,y) 映射到离散的栅格地图索引 (i,j)
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

void HybridAStar::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {//生成当前节点的所有合法邻居节点（包括不同档位的和不同方向的一串串的节点）
    
    neighbor_nodes.clear();

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {//i 遍历所有离散方向盘档位 phi → 离散的方向盘转角（steering angle）每个 phi 对应一条 可能的轨迹曲线
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();//车辆的后轴中心点
        double theta = curr_node_ptr->state_.z();//车辆当前状态

        const double phi = i * steering_radian_step_size_/*每一档之间的步长弧度*/;//用来离散化方向角的，phi是转向角

        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) {//沿着当前方向盘角 phi，离散地模拟向前一段轨迹的每一步
            DynamicModel(move_step_size_, phi, x, y, theta);//预测车辆从当前状态走一步之后的位置和朝向
            intermediate_state.emplace_back(Vec3d(x, y, theta));//将该状态存储

            if (!CheckCollision(x, y, theta)) {//检查该状态下是否与障碍物碰撞
                has_obstacle = true;
                break;
            }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());//碰撞前轨最最后一个点/轨迹最后一个点
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {//检查这个点是否在地图边界，且没有遇到障碍物
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();//邻居节点的最终状态（即轨迹末点 (x, y, θ)）
            neighbor_forward_node_ptr->steering_grade_ = i;//方向盘的档位
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;//前进方向
            neighbor_nodes.push_back(neighbor_forward_node_ptr);//加入邻居列表
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();//更新当前车辆状态
        for (int j = 1; j <= segment_length_discrete_num_; j++) {//沿着当前方向盘角 phi，离散地模拟向后一段轨迹的每一步
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {//小车离散更新方程，用来预测车辆从当前状态走一步之后的位置和朝向
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
    //wheel_base_ * std::tan(phi)是转弯半径，step_size / wheel_base_ * std::tan(phi)是转过的一个小角度
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);//x/2pai的余数

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

bool HybridAStar::BeyondBoundary(const Vec2d &pt) const {//判断给定的点有没有超过地图边界
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double HybridAStar:: ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {//当前节点到目标节点的启发式代价值h，用 A* 的 f = g + h 搜索策略
    double h;
    // L2
    // h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();//取起始节点到目标节点的曼哈顿距离作为代价值的初始估计

    if (h < 3.0 * shot_distance_) {//如果当前节点离目标很近时，简单距离可能不够精确，使用 Reeds-Shepp 最短路径
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }

    return h;
}

double HybridAStar::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {//计算车辆的运动代价
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {//前进分支
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {//如果转向等级（不同角度的转向）变化
            if (neighbor_node_ptr->steering_grade_ == 0) {//邻居节点的转向等级为 0，也就是 车辆处于直行状态，没有转向
                g = segment_length_ * steering_change_penalty_;//代价等于物理长度乘以转向等级变化惩罚系数
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;//如果是有转弯了，再乘以一个转弯惩罚
            }
        } else {//邻居节点与当前节点的转向等级相同
            if (neighbor_node_ptr->steering_grade_ == 0) {//如果是直行
                g = segment_length_;
            } else {//如果是弯道，加上弯道惩罚
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {//如果邻居节点是后退的
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {//如果转向等级变化了
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;//段长 × 转向变化惩罚 × 弯道惩罚 × 后退惩罚
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }
    //鼓励 前进优先，倒车代价更大
    // 鼓励 连续同转向，减少频繁换方向盘
    // 鼓励 直行，减少弯道

    return g;
}
//核心搜索函数
bool HybridAStar::Search(const Vec3d &start_state, const Vec3d &goal_state) {//输入起点状态和终点状态，返回true表示找到路径，返回false表示搜索失败
    Timer search_used_time;//记录搜索时间

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;//用于统计性能

    const Vec3i start_grid_index = State2Index(start_state);//将初始状态的坐标和角度离散化
    const Vec3i goal_grid_index = State2Index(goal_state);//将目标状态的坐标和角度离散化

    auto goal_node_ptr = new StateNode(goal_grid_index);//创建终点节点
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;//方向未知
    goal_node_ptr->steering_grade_ = 0;//转向等级为0

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;//设置连续状态
    start_node_ptr->steering_grade_ = 0;//转向等级
    start_node_ptr->direction_ = StateNode::NO;//方向未知
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;//加入开放表
    start_node_ptr->intermediate_states_.emplace_back(start_state);//保存连续轨迹点
    start_node_ptr->g_cost_ = 0.0;//从起点到当前节点的累积代价，当前为0
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);//计算h代价
    //f_cost = g + h
    // h 是启发式代价（估算从当前节点到目标的最短代价）
    // 起点没有 g_cost，所以 f_cost = h

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;//三维珊格映射表，每个格子存储一个指向statenode的指针
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();//openset_ 是搜索过程中用来存储待扩展节点的 开放列表
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {//在众多邻居中（一个网格只会有一个节点），选出代价最小的邻居，在将当前代价最小的作为当前节点
        current_node_ptr = openset_.begin()->second;//获取f——cost最小的元素的节点指针
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;//IN_CLOSESET 表示该节点 已被扩展 作用：后续生成的邻居节点，如果落在该栅格上，会被丢弃或更新
        openset_.erase(openset_.begin());//将已扩展的节点从开放表中删除，保持开放表只包含待扩展的节点

        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {//计算两点的欧几里德距离是否小于一定的距离
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                //每次从 current_node 展开若干邻居（离散前进、转向），但如果离终点很近了，就希望直接尝试 解析扩展 (Analytic Expansion) ——也就是用 Reeds-Shepp (RS) 曲线 从当前位置直接“连一条精确的可行曲线”到目标点
                terminal_node_ptr_ = goal_node_ptr;

                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;//从终点的父节点开始
                while (grid_node_ptr != nullptr) {//如果当前节点指针不为空
                    grid_node_ptr = grid_node_ptr->parent_node_;//不断追溯父节点
                    path_length_ = path_length_ + segment_length_;//每追溯一步，就把路径长度加上一个离散步长segment_length_，近似的
                }
                path_length_ = path_length_ - segment_length_ + rs_length;//先减掉最后那段假算的离散步长，再加上真正的 Reeds–Shepp 曲线长度。

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);//生成当前节点的所有合法邻居节点（包括不同档位的和不同方向的一串串的节点）
        neighbor_time = neighbor_time + timer_get_neighbor.End();//将本次耗时累加到历史总耗时中

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {//遍历每个邻居节点
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);//计算车辆运动代价（根据前进后退，转弯等级）
            compute_g_time = compute_g_time + timer_get_neighbor.End();//累积计算时间

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;//当前节点到目标节点的启发式代价值h，用 A* 的 f = g + h 搜索策略
            //tie_breaker_的作用
            //打破平局：原本相等的f代价现在有微小差异
            // 引导搜索：倾向于选择h值更小的节点（更接近目标）
            // 保持可采纳性：因为系数很接近1，几乎不影响最优性
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr->grid_index_;//节点在离散化状态空间中的网格坐标
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {//检查这个三维离散状态 (x, y, θ) 是否从未被探索过
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;//从起点到当前节点的已知最小代价+从当前节点到邻居节点的边代价（由ComputeG计算）
                neighbor_node_ptr->parent_node_ = current_node_ptr;//把当前节点设置为邻居节点的副节点
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr)/*key，value*/);//负责将节点按优先级插入开放集
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;//表示这个网格被特定节点访问过了
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {//该网格被发现，但是还没有被扩展
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;//临时代价计算，从起点到当前节点的已知最小代价+从当前节点到邻居节点的边代价（由ComputeG计算）

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {//比较: 新计算的代价 vs 原有节点记录的代价，如果新代价小: 说明通过当前节点到达该状态有更优路径
                    //节点信息更新
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;//节点仍在开放集中，等待后续扩展
                    //最终要选出代价最小的节点

                    /// TODO: This will cause a memory leak
                    //delete state_node_map_[index.x()][index.y()][index.z()];
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;//将映射表中的节点指针更新为新创建的节点，该网格位置现在指向包含更新信息的新节点对象
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;//释放内存
                continue;
            }
        }

        count++;
        if (count > 50000) {//检查最改迭代次数，防止无限循环
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }

    return false;
}

VectorVec4d HybridAStar::GetSearchedTree() {//存储最终返回的所有平滑路径段
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {//三重循环遍历状态网格
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {//不处理未探索节点或起点节点
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {//处理中间状态路径段
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);//intermediate_states_存储从父节点到当前节点的连续路径点，设置线段起点
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);//设置线段终点

                    tree.emplace_back(point_pair);//推到这颗树里面
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);//建立父节点到子节点的连接
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {//释放资源
    if (map_data_ != nullptr) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double HybridAStar::GetPathLength() const {//编译器指令: GNU扩展属性，告诉编译器该函数可能不会被使用。作用: 抑制"未使用函数"的警告信息
    return path_length_;
}

VectorVec3d HybridAStar::GetPath() const {//从终端节点回溯并构建完整的路径
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());//反转路径
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());//插入中间状态点
                    // path.end(): 插入位置 - 指向最后一个元素之后的位置
                    // node->intermediate_states_.begin(): 源容器的起始迭代器
                    // node->intermediate_states_.end(): 源容器的结束迭代器
    }

    return path;
}

void HybridAStar::Reset() {//负责重置混合A*算法的状态，但不释放主要内存结构
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {//解析扩展 - 在混合A*搜索过程中，尝试用Reeds-Shepp曲线直接连接当前节点与目标节点，实现"捷径"跳跃。
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);//存储离散的全局rs曲线轨迹点，也就是一个个端点的位姿

    for (const auto &pose: rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {//检查：这个点是否超出地图边界  是否在这个点的姿态下会与障碍物相撞
            return false;//超界或碰撞就返回false
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);//为了保证存储的路径点不重复

    return true;
}
