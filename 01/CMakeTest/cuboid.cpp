#include "cuboid.h"

double Cuboid::getDimensions() const
{

}

void Cuboid::setDimensions(double d, double w, double h)
{
    depth = d;
    width = w;
    height = h;
}

double Cuboid::getVolume() const
{
    return depth * width * height;
}
