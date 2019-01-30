#ifndef FEEDING_TARGETFOODRANKER_HPP_

namespace feeding {

// Base class for ranking target food items.
class TargetFoodRanker
{
public:

    /// Returns a sorted list of items.
    /// \param[in] items List of food items.
    /// \param[in] forqueTransform Pose of the forque tine.
    std::vector<const TargetFoodItem> sort(
        const std::vector<const TargetFoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;
};

} // namespace feeding

#endif