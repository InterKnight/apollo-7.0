/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  planner_open_space_config_.CopyFrom(open_space_conf);
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num();
  // 最大前轮转角是最大方向盘转角乘以一个系数
  max_steer_angle_ =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
  step_size_ = planner_open_space_config_.warm_start_config().step_size();
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config().traj_back_penalty();
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config().traj_steer_penalty();
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                   .traj_steer_change_penalty();
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  // 生成RS曲线
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    ADEBUG << "ShortestRSP failed";
    return false;
  }

  // 判断生成的RS曲线是否满足碰撞条件
  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  ADEBUG << "Reach the end configuration with Reed Sharp";
  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  // GT是greater than 的简写,数字后面加U表示用 unsigned int类型存储数字，默认是int，加U就可以减小空间消耗
  CHECK_GT(node->GetStepSize(), 0U);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  // 这里的node_step_size和全局的不一样，这里记录的是走了多少小步,应该叫node_step_num更贴切
  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  // 弄check_start_index这个变量出来是为了区分始末节点和中间节点
  // 所以叫 check_start_end_index 其实更贴切
  // 因为始末节点的node_step_size为1，而中间节点的node_step_size大于1（最小是2）
  // 而中间节点的第一步是不用做碰撞检测的，因为第一步就是原地不动的位置
  // 原地这个位置肯定是之前节点中计算过满足碰撞条件的，否则也不可能到这个位置来
  // 所以为了避免重复运算，引入了check_start_index
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  // 这里用for循环是因为，一个子节点在扩展过程中，可能是经过了几小步来的，这里
  // 要检查每一小步是否都满足条件
  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          ADEBUG << "collision start at x: " << linesegment.start().x();
          ADEBUG << "collision start at y: " << linesegment.start().y();
          ADEBUG << "collision end at x: " << linesegment.end().x();
          ADEBUG << "collision end at y: " << linesegment.end().y();
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;  // 这里是前轮转角，不是方向盘转角
  double traveled_distance = 0.0;
//首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的（前进和倒车）
  // next_node_num_ 默认是10，前一半是前进，后一半是后退，和下面的图有些出入，下面的图画的是6的情况
  // steering = 初始偏移量 + 单位间隔 × index
  /****************************************************************************
   *      转向定义为左打舵为 “+”，右打舵为负“-”，carsim里是这样定义的
   *      (-max_steer_angle_) 4   < \     / >   3  (max_steer_angle_)
   *                                 \   /
   *               (后退方向)  5 <----- O -----> 2  (前进方向)
   *                                 /   \
   *       (max_steer_angle_) 6   < /     \ >   1 (-max_steer_angle_)
   * **************************************************************************/
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    // 前进
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    // step_size_ 默认0.25，是每次向前运动的距离,为什么这个step是通过配置文件写死的，不是应该为 v*dt 吗？
    traveled_distance = step_size_;
  } else {
    // 后退
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  // 对角线长度，这样就能保证一个格子里，最多只有一个节点
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi(); // phi是车体朝向
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  // 在向外扩展子节点时，每次需要向外扩展arc长度，但不是一步就迈arc的长度，而是一小步一小步来，每一小步的长度为step_size_
  // 由于arc和grid resolution是根号2倍的关系，该node的next_node一定在相邻的其他grid内
  // 相邻grid内的最后一个路径点会被定义为next node，但该grid内可以有多个路径点，就是可以走多步
  // 如果step_size太大，则会不符合车辆动力学的简化假设，太小则会增加计算量。默认0.25
  // arc其实就是网格大小的根号2倍，arc太大可能轨迹不是最优的，arc太小会增加计算量. 网格大小默认0.3
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    /******************************************************************************
    * next_phi = last_phi + ω*dt      \
    *                                   ----> next_phi = last_phi + v/L*tan(δ)*dt
    * ω = v/R = v/L*tan(δ) 自行车模型  /                = last_phi + v*dt /L*tan(δ)
    *                                                  = last_phi + traveled_distance /L*tan(δ)
    *******************************************************************************/
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  // 感觉这里没有必要检查，因为在后面的ValidityCheck中还会检查是否出界了，这里感觉重复了？
  // 注释掉之后对总时间几乎没有影响，这段不太消耗资源
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  // node3d 有重载，可以直接用vector生成，会自动取vector中最后一个,并用step_size_这个
  // 类变量记录下，这最后一个是经过几小步来的。注意这里的step_size_和上面全局那个step_size_
  // 是不一样的，上面那个是通过配置文件给定的，记录的是每一小步走的距离，而node下的step_size_
  // 其实记录的是走了几小步，其实应该叫step_num_会更准确，不易混淆
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  // 子节点的历史代价 = 父节点的历史代价 + 父节点到子节点的代价
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  // 加了一个常量，不过这里是0，如果改大，则会加大启发代价，使的搜索过程更效率而轻质量
  double optimal_path_cost = 0.0;
  // 查表获得子节点到终点的距离
  // 在算法说明中，h应该取h1和h2之间较大的那个，但是这里只有h2，为什么？
  // h1是考虑动力学，不考虑障碍物；h2是考虑障碍物，不考虑动力学
  // 在没有障碍物的简单场景下测试，增加h1之后对总时间没有影响
  // h1=reeds_shepp_to_check->total_length
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

// 计算历史代价，将方向，转向等因素考虑其中，不是单纯的距离
double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    // 走的步数 * 每一步的距离 * 前向惩罚系数。减一是因为步数里包含原地那一步
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  // 切换方向的惩罚
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  // 转向惩罚，尽量不转向，默认是0
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  // 转向差值惩罚，转向插值尽量小,默认是0
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  // 只有start node 的pre node是nullptr
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "states sizes are not equal";
      return false;
    }
    // 将容器的顺序反转
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    // 这里去掉最后一个是因为和前一个节点的最后一个重复了
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  // 此时的current_node就是start node了
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  // 再反转一次变成正序
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  // 计算轨迹上每个点的速度，加速度，前轮转角
  if (!GetTemporalProfile(result)) {
    AERROR << "GetSpeedProfile from Hybrid Astar path fails";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal, "
           << "result->x.size(): " << result->x.size() << "result->y.size()"
           << result->y.size() << "result->phi.size()" << result->phi.size()
           << "result->v.size()" << result->v.size();
    return false;
  }
  // 因为终点没有a和steer，所以比x少1
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    AERROR << "control sizes not equal or not right";
    AERROR << " acceleration size: " << result->a.size();
    AERROR << " steer size: " << result->steer.size();
    AERROR << " x size: " << result->x.size();
    return false;
  }
  return true;
}

// 从轨迹中获取速度，加速度，前轮转角
bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  // 起点和终点的速度都是0, 这里的起点终点并不是整个轨迹的起点，终点
  // 而是每段轨迹的起点终点，因为是按前进后退来分段的，所以在前进和后退
  // 之间切换时，速度一定是0
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    // 这里其实是将速度分解为x向和y向的，然后再相加
    // 还可以用取平方和再开根号（hypot）的方式，可能运算量比求三角函数大把
    // 这里delta_t默认是0.5，那就代表着在apollo认为在delta_t的时间范围内，走了step_size的距离
    // 也就是在0.5s内，走了0.25m，这其实就能算速度是0.5m/s^2，这个显然不是一个普适的值
    // 然后他又用这个距离来反推速度，这显然是矛盾的？
    // 因为这些点不关是通过常规的扩展子节点而来，如果是这样来的，那根据之前的假设，
    // 速度确实是已知。但这些点还有可能通过RS曲线来，通过RS曲线来的点可能速度就是未知的。
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  // 每一段的终点没有加速度，因为会在下一段的起点给他赋值
  // 但这会造成，最后那段的终点，也就是整段轨迹的终点没有加速度
  // 为什么没有像速度那样取前后两个点，而是只取了前面的点？
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  // 这个steer是前轮转角，但是这又矛盾了，前轮转角前面已经通过最大前轮转角乘以系数后离散化得到了
  // 为什么还要求一遍？并且原来的的phi就是通过steer求的，现在又用phi求steer？
  // 因为这些点不关是通过常规的扩展子节点而来，如果是这样来的，那根据之前的假设，
  // 前轮转角确实是已知。但这些点还有可能通过RS曲线来，通过RS曲线来的点可能速度就是未知的。

  // 每一段的终点没有前轮转角，因为会在下一段的起点给他赋值
  // 但这会造成，最后那段的终点，也就是整段轨迹的终点没有前轮转角
    /******************************************************************************
    * next_phi = last_phi + ω*dt      \
    *                                   ----> next_phi = last_phi + v/L*tan(δ)*dt
    * ω = v/R = v/L*tan(δ) 自行车模型  /                = last_phi + v*dt /L*tan(δ)
    *                                                  = last_phi + step_size_ /L*tan(δ)
    *******************************************************************************/
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base() / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult* result) {
  // sanity check
  CHECK_NOTNULL(result);
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "result sizes not equal";
    return false;
  }

  // get gear info
  double init_heading = result->phi.front();
  const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                   result->y[1] - result->y[0]);
  const double gear =
      std::abs(common::math::NormalizeAngle(
          init_heading - init_tracking_vector.Angle())) < M_PI_2;

  // get path lengh
  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  for (size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }
  // assume static initial state
  const double init_v = 0.0;
  const double init_a = 0.0;

  // minimum time speed optimization
  // TODO(Jinyun): move to confs
  const double max_forward_v = 2.0;
  const double max_reverse_v = 1.0;
  const double max_forward_acc = 2.0;
  const double max_reverse_acc = 1.0;
  const double max_acc_jerk = 0.5;
  const double delta_t = 0.2;

  SpeedData speed_data;

  // TODO(Jinyun): explore better time horizon heuristic
  const double path_length = result->accumulated_s.back();
  const double total_t = std::max(gear ? 1.5 *
                                             (max_forward_v * max_forward_v +
                                              path_length * max_forward_acc) /
                                             (max_forward_acc * max_forward_v)
                                       : 1.5 *
                                             (max_reverse_v * max_reverse_v +
                                              path_length * max_reverse_acc) /
                                             (max_reverse_acc * max_reverse_v),
                                  10.0);

  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

  // set end constraints
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_acc = gear ? max_forward_acc : max_reverse_acc;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  // TODO(Jinyun): move to confs
  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR << "Piecewise jerk speed optimizer failed!";
    return false;
  }

  // extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // assign speed point by gear
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  const double kEpislon = 1.0e-6;
  const double sEpislon = 1.0e-6;
  for (size_t i = 1; i < num_of_knots; ++i) {
    if (s[i - 1] - s[i] > kEpislon) {
      ADEBUG << "unexpected decreasing s in speed smoothing at time "
             << static_cast<double>(i) * delta_t << "with total time "
             << total_t;
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
    // cut the speed data when it is about to meet end condition
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }

  // combine speed and path profile
  DiscretizedPath path_data;
  for (size_t i = 0; i < path_points_size; ++i) {
    common::PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_data.push_back(std::move(path_point));
  }

  HybridAStartResult combined_result;

  // TODO(Jinyun): move to confs
  const double kDenseTimeResoltuion = 0.5;
  const double time_horizon =
      speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
  if (path_data.empty()) {
    AERROR << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResoltuion) {
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data.Length()) {
      break;
    }

    common::PathPoint path_point = path_data.Evaluate(speed_point.s());

    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());
    if (!gear) {
      combined_result.v.push_back(-speed_point.v());
      combined_result.a.push_back(-speed_point.a());
    } else {
      combined_result.v.push_back(speed_point.v());
      combined_result.a.push_back(speed_point.a());
    }
  }

  combined_result.a.pop_back();

  // recalc step size
  path_points_size = combined_result.x.size();

  // load steering from phi
  for (size_t i = 0; i + 1 < path_points_size; ++i) {
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        vehicle_param_.wheel_base() /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    discrete_steer =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
    combined_result.steer.push_back(discrete_steer);
  }

  *result = combined_result;
  return true;
}

// 轨迹分段，方向相同的为一段，比如先前进再后退，这就是两段
// 分段后便于求得各段轨迹的速度，加速度，前轮转角
bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result";
    return false;
  }

  // 一共走了多少步
  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  // 起点 start node 的phi，也就是heading
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  // true为前进，false为后退
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    // 分段的时候，前一段的终点和后一段的起点是同一个点，也就是那一点存在于两段轨迹中
/*     0 -- 1 -- 2     [0, 1, 2] 是第一段
                /      [2, 3, 4] 是第二段
               /
              3
             /
            /
           4 
*/     
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  // 对最后一个特殊处理，因为它没有下一个了，不能放在循环里
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  // 从轨迹中获取速度，加速度和steer
  for (auto& result : *partitioned_result) {
    // 默认是false
    if (FLAGS_use_s_curve_speed_smooth) {
      // 用piecewise jerk来获取相应值，计算量更大
      if (!GenerateSCurveSpeedAcceleration(&result)) {
        AERROR << "GenerateSCurveSpeedAcceleration fail";
        return false;
      }
    } else {
      if (!GenerateSpeedAcceleration(&result)) {
        AERROR << "GenerateSpeedAcceleration fail";
        return false;
      }
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  // 从轨迹中获取速度，加速度和steer的时间，其实不止是速度
  ADEBUG << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
  return true;
}

// 将轨迹分段，计算速度，加速度，前轮转角后，再将轨迹拼接起来
bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  // 将轨迹分段，并且计算每个点的速度，加速度，前轮转角
  // 分段是为了更好计算每段的速度，加速度，前轮转角，因为每一段的
  // 衔接处的速度一定为0
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    AERROR << "TrajectoryPartition fail";
    return false;
  }
  // 将分成多段的轨迹再拼在一起
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    // 因为每一小段的终点都没有a和steer，所以不用减一
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  // 补一下终点的数据
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  *result = stitched_result;
  return true;
}

// Hybrid A star的核心过程
bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStartResult* result) {
  // clear containers
  //每次规划，清空之前的缓存数据
  open_set_.clear();
  close_set_.clear();
  // decltype,为什么一定要用这个,因为pq本身没有clear方法，要么用下面这种，要么自己用swap实现一个clear方法。
  // 其实不用它自动检测类型，类型是已知的，可能是为了写起来方便，因为那个类型特别长
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;
   //构造障碍物轮廓线段容器
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  // 拿到每个障碍物
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    // 角点数量
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    // 依次对该障碍物的每个边读取
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      // 遍历各个顶点得到边，这里应该少了end至start的情况
      // 推测少的原因是：会将目标车位的4条边也当成障碍物的
      // 轮廓进行碰撞检测。这里留一条边的原因就是让车可以从
      // 那条边进入车库？
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    // 得到该障碍物的所有边
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  // 这个是干嘛用的？右值引用
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
  // load XYbounds
  XYbounds_ = XYbounds;
  // load nodes and obstacles
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    AERROR << "start_node in collision with obstacles";
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    AERROR << "end_node in collision with obstacles";
    return false;
  }

  // f = g + h，在apollo中变量名称是：
  // cost_ = path_cost_ + heuristic_
  // 使用djkstra算法来计算目标点到图中任一点的path_cost
  // 相当于把地图里每个点都跑了一遍，知道了到每个点的距离
  // 为什么不用经典A*呢，因为经典A*适合点到点的最短距离，
  // 想起点到所有点的距离，用dijkstra更合适
  // dijkstra相当与把经典A*中 f=g+h中的h设为0，也就是不启发
  // 生成一个dp_map,这里的dp是动态规划的意思。这个dp_map是一个unordered map
  // 里面记录了一个二维网格地图中所有的格子，也就是所有节点，每一个节点上都有
  // 它到终点的path_cost，后续只需要查表，空间换时间
  // 如何保证这个方法只调用一次？
  double map_time = Clock::NowInSeconds();
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);
  ADEBUG << "map time " << Clock::NowInSeconds() - map_time;
  // load open set, pq
  // 用emplace比push效率高，并且用push的话，需要写成 make_pair(next_node->GetIndex(), next_node)，多一个make_pair
  // open_set_是一个unordered_map的类型，用这个是因为这种键值对的形式比较方便，不需要排序所以用unordered可能节省资源。
  // open_pq_现在是pq的类型，主要是要有序这个特性。虽然set也可以有序，但是pq比set快，set多了一个唯一的属性
  // open_pq_的排序规则是自定义的，是按cost排序的
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
  // GetIndex()返回的是一个string，类似 "x_y_phi"
  // Hybrid A* begins
  size_t explored_node_num = 0;
  double astar_start_time = Clock::NowInSeconds();
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    // 用RS曲线试试运气，运气爆棚可以到达终点，则搜索结束，理论上这不应该每次都试把？
    // 推测可能是因为每次开始用混合A*搜索路径时，车辆离车位已经很近了，默认5m左右，
    // 所以这时候应该很快能找到合适路径，每次都试也可以
    // 在简单无障碍物的环境下测试了一下，改成2个点算一次（explored_node_num % 2 == 0）后，
    // rs_time减小了，explored_node_num基本不变，总时间减小约20%
    // 但整个混合A*不稳定，多次跑的结果不一致，差距大于20%，所以改的意义也不大
    // 在简单无障碍物的环境下测试了一下，改成10个点算一次（explored_node_num % 10 == 0）后，
    // 无法计算出路径
    const double rs_start_time = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node)) {
      break;
    }
    const double rs_end_time = Clock::NowInSeconds();
    rs_time += rs_end_time - rs_start_time;
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      // 其实不止是碰撞检测，还有检测出界
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        const double start_time = Clock::NowInSeconds();
        CalculateNodeCost(current_node, next_node);
        const double end_time = Clock::NowInSeconds();
        // 这里叫heuristic_time其实不准确，这个过程包含两部分，f和h
        // 其中f需要计算，h直接查表
        heuristic_time += end_time - start_time;
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  // 这里final_node_只能通过RS曲线获得
  if (final_node_ == nullptr) {
    ADEBUG << "Hybrid A searching return null ptr(open_set ran out)";
    return false;
  }
  if (!GetResult(result)) {
    ADEBUG << "GetResult failed";
    return false;
  }
  // 在简单无障碍物的环境下测试了一下，耗时在1~2s之间存在波动，前3名分别是 AnalyticExpansion > Next_node_generator > ValidityCheck
  // 但随着障碍物数量的增多，ValidityCheck势必耗时会增加
  ADEBUG << "explored node num is " << explored_node_num;
  ADEBUG << "heuristic time is " << heuristic_time;
  ADEBUG << "reed shepp time is " << rs_time;
  ADEBUG << "hybrid astar total time is "
         << Clock::NowInSeconds() - astar_start_time;
  return true;
}
}  // namespace planning
}  // namespace apollo
