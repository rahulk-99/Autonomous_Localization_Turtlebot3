 
#include <part_string.hpp>
//switch statement to map the string color to its respective number provided in parts.msgs file
std::string part_color(uint8_t color){
  switch(color){
    case 0: return "Red";
    case 1: return "Green";
    case 2: return "Blue";
    case 3: return "Orange";
    case 4: return "Purple";
    default : return "no such part of this color";
  }
}
//switch statement to map the string type to its respective number provided in parts.msgs file
std::string part_type(uint8_t type){
  switch(type){
    case 10: return "Battery";
    case 11: return "Pump";
    case 12: return "Sensor";
    case 13: return "Regulator";
    default : return "no such part of this type";
   
  }
}