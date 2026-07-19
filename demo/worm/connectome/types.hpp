#pragma once

#include <cstdint>

namespace connectome {

using NeuronId = std::uint32_t;

// Соответствует делению нейронов в коннектомных датасетах (Cook et al. 2019,
// Varshney et al. 2011): сенсорные / интер / мотонейроны, часть из которых
// совмещает роли (например, RIML/RIMR — командные интернейроны реверса,
// типизированы как ProcessingOutput; AVA/AVB, при всей их командной роли,
// в данных типизированы просто как Processing).
enum class NeuronType : std::uint8_t {
    Input,             // ввод: чистый интерфейсный узел (сырое значение сенсора)
    InputProcessing,    // ввод/обработка: сенсорный нейрон со своей динамикой
    Processing,          // обработка: интернейрон
    ProcessingOutput,    // обработка/вывод: командный/двигательный нейрон со своей динамикой
    Output,              // вывод: чистый интерфейсный узел (значение для актуатора)
};

// Параметры узла-нейрона в редуцированной (не спайковой) модели утечки:
// C * dV/dt = -leak * (V - rest) + I_chem + I_gap + external
struct NeuronParams {
    float leak = 1.0f;      // проводимость утечки Gc
    float rest = 0.0f;      // потенциал покоя E_leak
    float capacitance = 1.0f; // C, тепловая инерция нейрона
};

} // namespace connectome
