#ifndef hsvrgb_hpp
#define hsvrgb_hpp

#include <iostream>
#include <cstdlib>
#include <cmath>

namespace hsvrgb
{
	void RGBtoHSV(double &fR, double &fG, double &fB, double &fH, double &fS, double &fV);
	void HSVtoRGB(double &fR, double &fG, double &fB, double &fH, double &fS, double &fV);
}

#endif // hsvrgb_hpp
