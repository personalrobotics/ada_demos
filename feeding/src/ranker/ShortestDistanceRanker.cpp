#include "feeding/ShortestDistanceRanker.hpp"
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
std::vector<std::size_t> getRanking(
    const std::vector<const TargetFoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const
{
    std::vector<std::size_t> ranks;
    ranks.reserve(items.size());

    std::vector<double> distancesToForque;
    for(const auto& item: items)
    {
        distancesToForque.emplace_back(
            getDistance(item.getPose(), forqueTransform));
    }

    // Sort based on the distance

    // TODO
    for(std::size_t i = 0; i < items.size(); ++i)
        ranks.emplace_back(i);

    return ranks;
}



} // namespace feeding