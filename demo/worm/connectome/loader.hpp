#pragma once

#include <string>
#include <vector>

#include "network.hpp"
#include "types.hpp"

namespace connectome {

// Метаданные мышцы брюшной стенки для узла Output: сторона тела и позиция
// вдоль передне-задней оси (1..24 для C. elegans), взятые прямо из имени
// клетки (MDL07 = Muscle Dorsal Left #7). is_muscle=false для всех
// остальных узлов (сенсорные/интер/командные нейроны).
struct MuscleInfo {
    bool is_muscle = false;
    char side = 0; // 'D' или 'V'
    int position = -1;
};

struct LoadedConnectome {
    Network network;
    std::vector<std::string> names;
    std::vector<MuscleInfo> muscles; // parallel to names/network state, size() == network.size()
};

// Загружает сеть из простого текстового формата, который производит
// data/convert/build_connectome.py из реальных данных коннектома (Cook et
// al. 2019 + Wang et al. 2024), см. data/README.md.
LoadedConnectome load_connectome(const std::string& path);

} // namespace connectome
