#ifndef IWR6843PARAM_H
#define IWR6843PARAM_H

#include <string>
#include <map>

class IWR6843AOP_TLV {
private:
    std::map<std::string, double> config_params;

public:
    IWR6843AOP_TLV(const std::string& config_file);
    std::map<std::string, double> get_params() const { return config_params; }
};

#endif // IWR6843PARAM_H