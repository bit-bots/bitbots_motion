/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <iomanip>
#include <stdexcept>
#include "bitbots_splines/Spline.hpp"

namespace bitbots_splines {

double Spline::pos(double t) const
{
    return interpolation(t, &Polynom::pos);
}
double Spline::vel(double t) const
{
    return interpolation(t, &Polynom::vel);
}
double Spline::acc(double t) const
{
    return interpolation(t, &Polynom::acc);
}
double Spline::jerk(double t) const
{
    return interpolation(t, &Polynom::jerk);
}
        
double Spline::posMod(double t) const
{
    return interpolationMod(t, &Polynom::pos);
}
double Spline::velMod(double t) const
{
    return interpolationMod(t, &Polynom::vel);
}
double Spline::accMod(double t) const
{
    return interpolationMod(t, &Polynom::acc);
}
double Spline::jerkMod(double t) const
{
    return interpolationMod(t, &Polynom::jerk);
}
        
double Spline::min() const
{
    if (_splines.empty()) {
        return 0.0;
    } else {
        return _splines.front().min;
    }
}
double Spline::max() const
{
    if (_splines.empty()) {
        return 0.0;
    } else {
        return _splines.back().max;
    }
}
        

        
void Spline::exportData(std::ostream& os) const
{
    for (const auto & _spline : _splines) {
        os << std::setprecision(17) << _spline.min << " ";
        os << std::setprecision(17) << _spline.max << " ";
        os << std::setprecision(17) << 
            _spline.polynom.getCoefs().size() << " ";
        for (size_t j=0;j<_spline.polynom.getCoefs().size();j++) {
            os << std::setprecision(17) << 
                _spline.polynom.getCoefs()[j] << " ";
        }
    }
    os << std::endl;
}
void Spline::importData(std::istream& is)
{
    bool isFormatError;
    while (is.good()) {
        isFormatError = true;
        double min;
        double max;
        size_t size;
        Polynom p;
        //Load spline interval and degree
        is >> min;
        if (!is.good()) break;
        is >> max;
        if (!is.good()) break;
        is >> size;
        //Load polynom coeficients
        p.getCoefs().resize(size);
        for (size_t i=0;i<size;i++) {
            if (!is.good()) break;
            is >> p.getCoefs()[i];
        }
        //Save spline part
        isFormatError = false;
        _splines.push_back({p, min, max});
        //Exit on line break
        while (is.peek() == ' ') {
            if (!is.good()) break;
            is.ignore();
        }
        if (is.peek() == '\n') {
            break;
        }
    }
    if (isFormatError) {
        throw std::logic_error(
            "Spline import format invalid");
    }
    //Call possible post import
    importCallBack();
}
        
size_t Spline::size() const
{
    return _splines.size();
}
        
const Spline::Spline_t& Spline::part(size_t index) const
{
    return _splines.at(index);
}
        
void Spline::addPart(const Polynom& poly, 
    double min, double max)
{
    _splines.push_back({poly, min, max});
}
        
void Spline::copyData(const Spline& sp)
{
    _splines = sp._splines;
    //Call possible post import
    importCallBack();
}
        
void Spline::importCallBack()
{
}

double Spline::interpolation(double x, 
    double(Polynom::*func)(double) const) const
{
    //Empty case
    if (_splines.empty()) {
        return 0.0;
    }
    //Bound asked abscisse into spline range
    if (x <= _splines.front().min) {
        x = _splines.front().min;
    }
    if (x >= _splines.back().max) {
        x = _splines.back().max;
    }
    //Bijection spline search
    size_t indexLow = 0;
    size_t indexUp = _splines.size()-1;
    while (indexLow != indexUp) {
        size_t index = (indexUp+indexLow)/2;
        if (x < _splines[index].min) {
            indexUp = index-1;
        } else if (x > _splines[index].max) {
            indexLow = index+1;
        } else {
            indexUp = index;
            indexLow = index;
        }
    }
    //Compute and return spline value
    return (_splines[indexUp].polynom.*func)
        (x-_splines[indexUp].min);
}

double Spline::interpolationMod(double x, 
    double(Polynom::*func)(double) const) const
{
    if (x < 0.0) {
        x = 1.0 + (x - ((int)x/1));
    } else if (x > 1.0) {
        x = (x - ((int)x/1));
    }
    return interpolation(x, func);
}

}

