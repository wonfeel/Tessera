#include "loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace connectome {

namespace {

// Не бросает исключение на EOF - нужно, чтобы отличить "реальная ошибка
// формата" от "файл кончился, а это была последняя (необязательная) секция"
// (см. PEPTIDE_EDGES ниже - старые файлы коннектома его не содержат).
bool try_next_line(std::ifstream& in, std::string& out) {
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        out = line;
        return true;
    }
    return false;
}

std::string next_line(std::ifstream& in) {
    std::string line;
    if (!try_next_line(in, line)) throw std::runtime_error("connectome file ended unexpectedly");
    return line;
}

NeuronType parse_type(const std::string& s) {
    static const std::unordered_map<std::string, NeuronType> kMap = {
        {"Input", NeuronType::Input},
        {"InputProcessing", NeuronType::InputProcessing},
        {"Processing", NeuronType::Processing},
        {"ProcessingOutput", NeuronType::ProcessingOutput},
        {"Output", NeuronType::Output},
    };
    auto it = kMap.find(s);
    if (it == kMap.end()) throw std::runtime_error("unknown neuron type in connectome file: " + s);
    return it->second;
}

} // namespace

LoadedConnectome load_connectome(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open connectome file: " + path);

    std::string tag;
    NeuronId node_count = 0;
    {
        std::istringstream header(next_line(in));
        header >> tag >> node_count;
        if (tag != "NODES") throw std::runtime_error("expected NODES header, got: " + tag);
    }

    std::vector<NeuronType> types(node_count);
    std::vector<NeuronParams> params(node_count);
    std::vector<std::string> names(node_count);
    std::vector<MuscleInfo> muscles(node_count);

    for (NeuronId i = 0; i < node_count; ++i) {
        std::istringstream ls(next_line(in));
        std::string name, type_str, side_str;
        int pos = -1;
        ls >> name >> type_str >> side_str >> pos;
        names[i] = name;
        types[i] = parse_type(type_str);
        params[i] = NeuronParams{};
        if (side_str.size() == 1 && (side_str[0] == 'D' || side_str[0] == 'V')) {
            muscles[i] = MuscleInfo{true, side_str[0], pos};
        }
    }

    std::size_t chem_count = 0;
    {
        std::istringstream header(next_line(in));
        header >> tag >> chem_count;
        if (tag != "CHEM_EDGES") throw std::runtime_error("expected CHEM_EDGES header, got: " + tag);
    }
    std::vector<CsrMatrix::Edge> chem_edges;
    chem_edges.reserve(chem_count);
    for (std::size_t i = 0; i < chem_count; ++i) {
        std::istringstream ls(next_line(in));
        NeuronId s, t;
        float w;
        ls >> s >> t >> w;
        chem_edges.push_back(CsrMatrix::Edge{s, t, w});
    }

    std::size_t gap_count = 0;
    {
        std::istringstream header(next_line(in));
        header >> tag >> gap_count;
        if (tag != "GAP_EDGES") throw std::runtime_error("expected GAP_EDGES header, got: " + tag);
    }
    std::vector<CsrMatrix::Edge> gap_edges;
    gap_edges.reserve(gap_count * 2);
    for (std::size_t i = 0; i < gap_count; ++i) {
        std::istringstream ls(next_line(in));
        NeuronId a, b;
        float w;
        ls >> a >> b >> w;
        gap_edges.push_back(CsrMatrix::Edge{a, b, w});
        gap_edges.push_back(CsrMatrix::Edge{b, a, w}); // симметрируем: gap junction двунаправлен
    }

    // PEPTIDE_EDGES - необязательная секция (Ripoll-Sánchez et al. 2023,
    // Neuron - предсказанная GPCR-лиганд связность PDF-1/PDFR-1, mid-range
    // модель; см. Network::set_peptide_connectivity). Бинарная - вес не
    // хранится в файле, каждое ребро получает 1.0f. Старые файлы коннектома
    // без этой секции читаются как есть (пустая связность, gain=0 всё равно
    // делает её нулевой - см. network.hpp).
    std::vector<CsrMatrix::Edge> peptide_edges;
    std::vector<char> has_outgoing_peptide(static_cast<std::size_t>(node_count), 0);
    std::string peptide_header;
    if (try_next_line(in, peptide_header)) {
        std::size_t peptide_count = 0;
        std::istringstream header(peptide_header);
        header >> tag >> peptide_count;
        if (tag != "PEPTIDE_EDGES") throw std::runtime_error("expected PEPTIDE_EDGES header, got: " + tag);
        peptide_edges.reserve(peptide_count);
        for (std::size_t i = 0; i < peptide_count; ++i) {
            std::istringstream ls(next_line(in));
            NeuronId s, t;
            ls >> s >> t;
            peptide_edges.push_back(CsrMatrix::Edge{s, t, 1.0f});
            has_outgoing_peptide[static_cast<std::size_t>(s)] = 1;
        }
    }
    std::vector<NeuronId> peptide_source_ids;
    for (NeuronId i = 0; i < node_count; ++i) {
        if (has_outgoing_peptide[static_cast<std::size_t>(i)]) peptide_source_ids.push_back(i);
    }

    Network network(types, params, CsrMatrix::from_edges(node_count, chem_edges),
                     CsrMatrix::from_edges(node_count, gap_edges));
    network.set_peptide_connectivity(CsrMatrix::from_edges(node_count, peptide_edges),
                                      std::move(peptide_source_ids));

    return LoadedConnectome{
        std::move(network),
        std::move(names),
        std::move(muscles),
    };
}

} // namespace connectome
