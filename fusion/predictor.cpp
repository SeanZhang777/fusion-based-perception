#include "predictor.h"

namespace kit {
namespace perception {
namespace fusion {

bool Predictor::Predict(const FusionObjectListPtr &fusion_obj_list, double ts) {
    // TODO: Predict global objects to local timestamp
    size_t num_objs = fusion_obj_list->objs.size();
    if (num_objs <= 0) {
        return true;
    }
    double time_diff = ts - fusion_obj_list->time_ns;
    for (size_t i = 0; i < num_objs; ++i) {
        auto &obj = fusion_obj_list->objs.at(i);
        obj->x += obj->velo_x * time_diff;
        obj->y += obj->velo_y * time_diff;
        obj->z += obj->velo_z * time_diff;
    }
    return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
