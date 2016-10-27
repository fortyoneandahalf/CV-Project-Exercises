#ifndef CUBOID_H
#define CUBOID_H


class Cuboid
{
private:
    double depth, width, height;

//    double calculateVolume() const;

public:
    Cuboid() {}
    Cuboid(double d, double w, double h)
    {
        setDimensions(d, w, h);
    }

    double getDimensions() const;
    void setDimensions(double, double, double);

    double getVolume() const;

};

#endif // CUBOID_H
