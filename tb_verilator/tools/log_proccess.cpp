
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>
#include <iomanip>

#define ENTRY_ADDR_STR "80000000"
#define ENTRY_ADDR 0x80000000

using namespace std;

map<string, unsigned int> func_entry_map; /* function name and its entry address */
map<unsigned int, string> instr; /* pc and instruction */
map<unsigned int, string> instr_func_map; /* pc and its function */

bool compress = true;

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(),
            s.end(), [](unsigned char c) { return !std::isxdigit(c); }) == s.end();
}


void parse_function_interval(fstream &obj_f)
{
    string line_buffer;
    string func_name = "";
    while (getline(obj_f, line_buffer)) {
        if (line_buffer.size() > 8) {
            string addr_str = line_buffer.substr(0, 8);
            for (char &c : addr_str) {
                if (c == ' ') c = '0';
            }
            /* make sure the line is instruction or function entry */
            if (is_number(addr_str)) {
                unsigned int addr_val = stoul(addr_str, nullptr, 16);

                if (line_buffer[8] == ':') { 
                    /* the line is instruction */
                    string instruction = line_buffer.substr(10); //to end
                    instr[addr_val] = instruction;
                    instr_func_map[addr_val] = func_name;
                    //cout << "dump " << setw(8) << hex << right << addr_val << " " << instruction_contain << endl;
                } else {
                    /* the line is function entry */
                    func_name = line_buffer.substr(9);
                    func_entry_map[func_name] = addr_val;
                }
            }
        }
    }
}


int main(int argc, char* argv[])
{
    if (argc < 3){
        cerr << "[usage] ./log_proccess <log_file> <objdump_files>" << endl;
        return -1;
    }

    string log_filename(argv[1]);
    fstream log_f(log_filename,fstream::in);
    if (!log_f.is_open()){
        cerr << "failed to open \"" << log_filename << "\" !!! " <<endl;
        return -1;
    }

    for (int i = 2; i < argc; i++) {
        string objdump_filename(argv[i]);
        fstream obj_f(objdump_filename, fstream::in);
        if (!obj_f.is_open()){
            cerr << "failed to open \"" << objdump_filename << "\" !!! " <<endl;
            return -1;
        }
        parse_function_interval(obj_f);
        obj_f.close();
    }

    fstream dec_log("dec_cpu.log",fstream::out);
    if (!dec_log.is_open()){
        cerr << "failed to open \"dec_cpu.log\" !!! " <<endl;
        return -1;
    }
    
    fstream func_log("func.log", fstream::out);
    if (!func_log.is_open()) {
        cerr << "failed to open \"func.log\" !!! " <<endl;
        return -1;
    }



    int cumulation = 1;
    unsigned int prev_instr_addr = 0;
    map<unsigned int,string>::iterator it;

    /* for (auto p : func_intervals) { */
    /*     if (p.second.first == curr_interval_start) { */ 
    /*         curr_func = p.first; */
    /*         curr_interval_end = p.second.second; */ 
    /*         cout << p.first << ":   "; */
    /*         cout << hex << curr_interval_start << ", " << curr_interval_end << dec << endl; */
    /*     } */
    /* } */

    stringstream log_buffer;
    string line_buffer;
    string prev_func_name;
    while (getline(log_f, line_buffer)) {
        if (line_buffer[0] == '#') {
            unsigned int clk = stoul(line_buffer.substr(1, 10), nullptr, 16);
            unsigned int addr_val = stoul(line_buffer.substr(12, 8), nullptr, 16);

            if (addr_val == 0) continue;

            if (prev_instr_addr != addr_val) {
                cumulation = 1;
                prev_instr_addr = addr_val;
               
                string cur_func_name = instr_func_map[addr_val];
                if (cur_func_name != prev_func_name) {
                    prev_func_name = cur_func_name;
                    /* branch to other function */
                    unsigned int entry_offset = addr_val - func_entry_map[cur_func_name];
                    dec_log << "function_entry:" << setfill('0') << setw(8) << right << hex
                        << addr_val << cur_func_name << "+ 0x" << entry_offset << endl;
                    func_log << "function_entry:" << setfill('0') << setw(8) << right << hex
                        << addr_val << cur_func_name << "+ 0x" << entry_offset << endl;
                }
                dec_log << "instruction:"
                    << setfill('0') << setw(8) << right << hex << addr_val << ":    " << instr[addr_val] << endl;
            } else {
                cumulation++;
            }
        }
    }
    log_f.close();
    dec_log.close();
    func_log.close();
    return 0;
}

