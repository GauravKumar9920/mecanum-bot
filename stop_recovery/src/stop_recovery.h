#ifndef STOP_RECOVERY_STOP_RECOVERY_H
#define STOP_RECOVERY_STOP_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace stop_recovery
{
/**
 * @class StopRecovery
 * @brief A recovery behavior that stops the robot in-place to attempt to clear out space
 */
class StopRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  StopRecovery();
  StopRecovery(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);
  /**
   * @brief  Initialization function for the StopRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the StopRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the stop recovery behavior
   */
  ~StopRecovery();

private:
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  //double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  base_local_planner::CostmapModel* world_model_;
};
};  // namespace stop_recovery
#endif  // STOP_RECOVERY_STOP_RECOVERY_H
