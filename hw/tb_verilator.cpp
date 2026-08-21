//------------------------------------------------------------------------------
// File: tb_verilator.cpp
// Description: High-speed C++ testbench for running the FSLAM ORB accelerator
//              compiled with Verilator.
//------------------------------------------------------------------------------

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

    std::cout << "================================================================" << std::endl;
    std::cout << " Running ORB Accelerator on Verilator (C++ Cycle-Accurate Model)" << std::endl;
    std::cout << "================================================================" << std::endl;
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
    top->s_axi_awaddr = 0;
    top->s_axi_wdata = 0;
    top->s_axi_awvalid = 0;
    top->s_axi_wvalid = 0;
    top->s_axi_araddr = 0;
    top->s_axi_arvalid = 0;
    top->s_axi_bready = 1;
    top->s_axi_rready = 1;

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
    for (int i = 0; i < 20; i++) tick();
    top->aresetn = 1;
    for (int i = 0; i < 20; i++) tick();

    // Enable accelerator via AXI4-Lite (Reg 0x00 = 1)
    std::cout << "[TB] Enabling accelerator via AXI4-Lite..." << std::endl;
    top->s_axi_awaddr = 0x00;
    top->s_axi_wdata = 0x01;
    top->s_axi_awvalid = 1;
    top->s_axi_wvalid = 1;
    while (!top->s_axi_awready) tick();
    tick();
    top->s_axi_awvalid = 0;
    top->s_axi_wvalid = 0;
    for (int i = 0; i < 10; i++) tick();

    // Streaming pixels & packet capture
    std::cout << "[TB] Streaming pixels and capturing features..." << std::endl;
    size_t word_send_idx = 0;
    int packet_word_idx = 0;
    uint32_t captured_packet[10] = {0};
    int extracted_features = 0;

    while (word_send_idx < pixel_words.size() || main_time < 3000000) {
        // Feed AXI-Stream slave
        if (word_send_idx < pixel_words.size()) {
            top->s_axis_tdata = pixel_words[word_send_idx];
            top->s_axis_tvalid = 1;
            top->s_axis_tlast = (word_send_idx == pixel_words.size() - 1);
            if (top->s_axis_tready) {
                word_send_idx++;
            }
        } else {
            top->s_axis_tvalid = 0;
            top->s_axis_tlast = 0;
        }

        tick();

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
    std::cout << "================================================================" << std::endl;
    std::cout << "[TB] Verilator Simulation Complete!" << std::endl;
    std::cout << "[TB] Total Features Extracted: " << extracted_features << std::endl;
    std::cout << "================================================================" << std::endl;

    delete top;
    return 0;
}
