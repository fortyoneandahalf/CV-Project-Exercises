#include <iostream>
#include "cuboid.h"

using namespace std;

double inputNumber()
{
    double d;

    cout << "Input number: ";
    cin >> d;
}


/* Question 1 */
int question1(void)
{
    double d1;
    double d2;

    d1 = inputNumber();
    d2 = inputNumber();

    cout << "Addition: " << (d1 + d2) << endl;
    cout << "Subtraction: " << (d1 - d2) << endl;
    cout << "Multiplication: " << (d1 * d2) << endl;

    return 0;
}


/* Question 2a */
int question2a(void)
{
    double d1;
    double d2;

    d1 = inputNumber();
    d2 = inputNumber();

    if(d1 > d2)
        cout << "First number is larger" << endl;
    else if(d2 > d1)
        cout << "Second number is larger" << endl;
    else
        cout << "Numbers are equal" << endl;

    return 0;
}


/* Question 2b */
int question2b(void)
{
    char selection;

    cout << "Please select an option:" << endl;
//    cout << "  <c> Start calculation" << endl;
//    cout << "  <s> Start program" << endl;
//    cout << "  <t> Terminate program" << endl;

    cin >> selection;

    switch (selection) {
    case 'c':
        cout << "Start calculation..." << endl;
        break;
    case 's':
        cout << "Start program..." << endl;
        break;
    case 't':
        cout << "Terminate program..." << endl;
        break;
    default:
        cout << "Unspecified input";
        break;
    }
}


int main(int argc, char *argv[])
{
    //return question1();
    //return question2a();
    //return question2b();

    Cuboid c;
    double d, w, h;

    cout << "Depth: ";
    cin >> d;
    cout << "Width: ";
    cin >> w;
    cout << "Height: ";
    cin >> h;

    c.setDimensions(d, w, h);

    cout << "Cuboid volume: " << c.getVolume() << endl;
}
