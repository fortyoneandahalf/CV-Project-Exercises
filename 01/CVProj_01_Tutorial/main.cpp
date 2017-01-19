#include <iostream>
#include <vector>
#include "cuboid.h"

//using namespace std;

double inputNumber()
{
    double d;

    std::cout << "Input number: ";
    std::cin >> d;
}


/* Question 1 */
int question1(void)
{
    double d1;
    double d2;

    d1 = inputNumber();
    d2 = inputNumber();

    std::cout << "Addition: " << (d1 + d2) << std::endl;
    std::cout << "Subtraction: " << (d1 - d2) << std::endl;
    std::cout << "Multiplication: " << (d1 * d2) << std::endl;

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
        std::cout << "First number is larger" << std::endl;
    else if(d2 > d1)
        std::cout << "Second number is larger" << std::endl;
    else
        std::cout << "Numbers are equal" << std::endl;

    return 0;
}


/* Question 2b */
int question2b(void)
{
    char selection;

    std::cout << "Please select an option:" << std::endl;
//    std::cout << "  <c> Start calculation" << std::endl;
//    std::cout << "  <s> Start program" << std::endl;
//    std::cout << "  <t> Terminate program" << std::endl;

    std::cin >> selection;

    switch (selection) {
    case 'c':
        std::cout << "Start calculation..." << std::endl;
        break;
    case 's':
        std::cout << "Start program..." << std::endl;
        break;
    case 't':
        std::cout << "Terminate program..." << std::endl;
        break;
    default:
        std::cout << "Unspecified input";
        break;
    }

    return 0;
}


/* Question 4 */
int question4(void)
{
    std::vector<int> vec;
    int input;

    for(int i=0; i<3; i++)
    {
        std::cout << "Input integer number " << i+1 << ": " << std::endl;
        std::cin >> input;
        vec.push_back(input);
    }

    std::cout << "Inputs in reverse order: ";

    for(int i=vec.size()-1; i>=0; i--)
    {
        std::cout << vec[i] << " ";
    }

    std::cout << std::endl;

    return 0;
}


int main(int argc, char *argv[])
{
    //return question1();
    //return question2a();
    //return question2b();

    // Question 3
//    Cuboid c;
//    double d, w, h;

//    std::cout << "Depth: ";
//    std::cin >> d;
//    std::cout << "Width: ";
//    std::cin >> w;
//    std::cout << "Height: ";
//    std::cin >> h;

//    c.setDimensions(d, w, h);

//    std::cout << "Cuboid volume: " << c.getVolume() << std::endl;

    return question4();
}
