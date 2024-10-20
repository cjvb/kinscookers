/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/
 
#include "ColorConverterLib.h"

void ColorConverter::RgbToHsv(uint8_t red, uint8_t green, uint8_t blue, double& hue, double& saturation, double& value)
{
	auto rd = static_cast<double>(red) / 255;
	auto gd = static_cast<double>(green) / 255;
	auto bd = static_cast<double>(blue) / 255;
	auto max = threeway_max(rd, gd, bd), min = threeway_min(rd, gd, bd);
	 
	value = max;

	auto d = max - min;
	saturation = max == 0 ? 0 : d / max;

	hue = 0;
	if (max != min)
	{
		if (max == rd)
		{
			hue = (gd - bd) / d + (gd < bd ? 6 : 0);
		}
		else if (max == gd)
		{
			hue = (bd - rd) / d + 2;
		}
		else if (max == bd)
		{
			hue = (rd - gd) / d + 4;
		}
		hue /= 6;
	}
}


void ColorConverter::TemperatureToRgb(int kelvin, uint8_t& red, uint8_t& green, uint8_t& blue)
{
	auto temp = kelvin / 100;

	if (temp <= 66)
	{
		red = 255;
		green = 99.4708025861 * log(temp) - 161.1195681661;

		if (temp <= 19)
		{
			blue = 0;
		}
		else
		{
			blue = 138.5177312231 * log(temp - 10) - 305.0447927307;
		}
	}
	else
	{
		red = 329.698727446 * pow(temp - 60, -0.1332047592);
		green = 288.1221695283 * pow(temp - 60, -0.0755148492);
		blue = 255;
	}
}

void ColorConverter::HexToRgb(String hex, uint8_t& red, uint8_t& green, uint8_t& blue)
{
	long number;
	if(hex[0] == '#')  number = strtol(&hex[1], nullptr, 16);
	else number = strtol(&hex[0], nullptr, 16);
	red = number >> 16;
	green = number >> 8 & 0xFF;
	blue = number & 0xFF;
}


double inline ColorConverter::threeway_max(double a, double b, double c)
{
	return max(a, max(b, c));
}

double inline ColorConverter::threeway_min(double a, double b, double c)
{
	return min(a, min(b, c));
}

double ColorConverter::hue2rgb(double p, double q, double t)
{
	if (t < 0) t += 1;
	if (t > 1) t -= 1;
	if (t < 1 / 6.0) return p + (q - p) * 6 * t;
	if (t < 1 / 2.0) return q;
	if (t < 2 / 3.0) return p + (q - p) * (2 / 3.0 - t) * 6;
	return p;
}

