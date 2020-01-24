// Detect Food
#include "feeding/action/DetectFood.hpp"

namespace feeding {
namespace action {
std::vector<std::unique_ptr<FoodItem>> detectFood(
    const std::shared_ptr<Perception>& perception,
    const std::string& foodName)
{
    while (true)
    {
        // Perception returns the list of good candidates, any one of them is good.
        // Multiple candidates are preferrable since planning may fail.
        candidateItems = perception->perceiveFood(foodName);

        if (candidateItems.size() == 0)
        {
        talk("I can't find that food. Try putting it on the plate.");
        ROS_WARN_STREAM(
            "Failed to detect any food. Please place food on the plate.");
        }
        else
        break;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    ROS_INFO_STREAM("Detected " << candidateItems.size() << " " << foodName);
    return candidateItems;
}
} // namespace action
} // namespace feeding