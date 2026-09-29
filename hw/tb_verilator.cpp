// Verilator testbench for hw_top: streams input_pixels.hex and writes the
// output features to rtl_features_verilator.txt.

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstdint>

#include "Vhw_top.h"
#include "verilated.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vhw_top* top = new Vhw_top;

    std::string hex_path = "input_pixels.hex";
    if (argc > 1) hex_path = argv[1];

    std::ifstream infile(hex_path);
    if (!infile.is_open()) {
        std::cerr << "[ERROR] Failed to open " << hex_path << std::endl;
        return 1;
    }

    std::vector<uint32_t> pixel_words;
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty()) continue;
        uint32_t word = std::stoul(line, nullptr, 16);
        pixel_words.push_back(word);
    }
    infile.close();

    std::cout << "[TB] Loaded " << pixel_words.size() << " 32-bit pixel words ("
              << pixel_words.size() * 4 << " pixels)" << std::endl;

    std::ofstream outfile("rtl_features_verilator.txt");

    // Initialize signals
    top->aclk = 0;
    top->aresetn = 0;
    top->s_axis_tdata = 0;
    top->s_axis_tvalid = 0;
    top->s_axis_tlast = 0;
    top->m_axis_tready = 1;

    uint64_t main_time = 0;
    auto tick = [&]() {
        top->aclk = 0;
        top->eval();
        main_time += 5;
        top->aclk = 1;
        top->eval();
        main_time += 5;
    };

    // Reset sequence
    for (int i = 0; i < 4; i++) tick();
    top->aresetn = 1;
    for (int i = 0; i < 4; i++) tick();

    // Streaming pixels & packet capture
    std::cout << "[TB] Streaming pixels and capturing features..." << std::endl;
    size_t word_send_idx = 0;
    int packet_word_idx = 0;
    uint32_t captured_packet[10] = {0};
    int extracted_features = 0;
    uint64_t cycle_count = 0;
    const uint64_t max_cycles = 1000000;

    while ((word_send_idx < pixel_words.size() || !top->__SYM__interrupt) &&
           cycle_count < max_cycles) {
        if (word_send_idx < pixel_words.size()) {
            top->s_axis_tdata = pixel_words[word_send_idx];
            top->s_axis_tvalid = 1;
            top->s_axis_tlast = (word_send_idx == pixel_words.size() - 1);
        } else {
            top->s_axis_tdata = 0;
            top->s_axis_tvalid = 0;
            top->s_axis_tlast = 0;
        }

        top->aclk = 0;
        top->eval();
        main_time += 5;

        bool axis_handshake = (top->s_axis_tvalid && top->s_axis_tready);

        top->aclk = 1;
        top->eval();
        main_time += 5;
        cycle_count++;

        if (axis_handshake) {
            word_send_idx++;
        }

        // Capture AXI-Stream master feature packets
        if (top->m_axis_tvalid && top->m_axis_tready) {
            captured_packet[packet_word_idx] = top->m_axis_tdata;
            if (packet_word_idx == 9 || top->m_axis_tlast) {
                extracted_features++;
                packet_word_idx = 0;

                uint32_t w0 = captured_packet[0];
                uint32_t w1 = captured_packet[1];
                uint32_t score = w0 & 0xFFFF;
                uint32_t ori = (w0 >> 16) & 0x3F;
                uint32_t lvl = (w0 >> 22) & 0x03;
                uint32_t x = w1 & 0x0FFF;
                uint32_t y = (w1 >> 16) & 0x0FFF;

                outfile << "FEAT: level=" << lvl
                        << " ori=" << ori
                        << " score=" << score
                        << " x=" << x
                        << " y=" << y
                        << " desc=" << std::hex << std::setfill('0');
                for (int w = 9; w >= 2; w--) {
                    outfile << std::setw(8) << captured_packet[w];
                }
                outfile << std::dec << std::endl;
            } else {
                packet_word_idx++;
            }
        }
    }

    outfile.close();
    std::cout << "[TB] Done: " << extracted_features << " features" << std::endl;

    delete top;
    return 0;
}
