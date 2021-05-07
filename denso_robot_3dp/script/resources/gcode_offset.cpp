/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>

using namespace std;

//const float zeroPt[] = {153, 48, 0};
//const float zeroPt[] = {133.28, 48, 0};
const float zeroPt[] = {133.5, 48.5, 0};
//G0 X133.5 Y48.5 Z0

void offset_pos_gcode(std::string &str) {
  // only offset if its x and y we have to rearange
  if ((str.substr(0, 2) == "G1" || str.substr(0, 2) == "G0") &&
      str.find('X') != -1) {
    int xStr[2], yStr[2];

    for (int i(0); i < str.length(); ++i) {
      if (str[i] == 'X') {
        // go to next next space or if end is reached first
        xStr[0] = i + 1;
        for (; str[i] != ' '; ++i) {
        }
        xStr[1] = i - xStr[0];

      } else if (str[i] == 'Y') {
        yStr[0] = i + 1;
        int yEnd = str.substr(yStr[0]).find_first_of(' ');
        // std::cout<<yEnd;
        yStr[1] = (yEnd != -1) ? yEnd : str.length() - yStr[0];
      }
    }
    // std::cout<<xStr[0]<<","<<xStr[1]<<" "<<yStr[0]<<","<<yStr[1]<<std::endl;
    // std::cout<<str.substr(xStr[0],xStr[1])<<"
    // "<<str.substr(yStr[0],yStr[1])<<std::endl;
    float xPos, yPos;
    // swich the x and y and apply offset
    xPos = std::stof(str.substr(yStr[0], yStr[1])) + zeroPt[0];
    yPos = zeroPt[1]- std::stof(str.substr(xStr[0], xStr[1]));
    // std::cout<<xPos<<std::endl;
    // std::cout<<yPos<<std::endl;
    str = str.substr(0, xStr[0]) + std::to_string(xPos) + " Y" +
          std::to_string(yPos) + str.substr(yStr[0] + yStr[1]);
  }
}

int main()
{
    string a = "G1 X69.205 Y46.728 Z0.300";
    offset_pos_gcode(a);
    cout<<"a is"<<a;

    return 0;
}



