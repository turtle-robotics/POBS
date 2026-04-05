#ifndef PARAM_EXPRESSIONS_HPP
#define PARAM_EXPRESSIONS_HPP

#include <vector>
#include <functional>

//A abstract template class that the others share functions with
//The reason this exists is to allow the changing of estimation functions in runtime
struct Expression
{
    double evaluate(double x);
};

//A quadratic estimation function
struct QuadraticExpression : Expression
{
    // ax^2+bx+c=y
    double a;
    double b;
    double c;

    QuadraticExpression(double a, double b, double c) : a(a), b(b), c(c) {}

    double evaluate(double x)
    {
        return a * x * x + b * x + c;
    }
};

//A quadratic estimation function however we can set a bound for the functions.
struct CheckedQuadraticExpression : QuadraticExpression
{
    
    std::function<double(double)> check;

    CheckedQuadraticExpression(double a, double b, double c, std::function<double(double)> check) : QuadraticExpression(a, b, c), check(check) {}

    double evaluate(double x)
    {
        double eval = a * x * x + b * x + c;
        return check(eval);
    }
};

//A linear estimation function.
struct LinearExpression : Expression
{
    // ax+b=y
    double a;
    double b;

    LinearExpression(double a, double b) : a(a), b(b) {}

    double evaluate(double x)
    {
        return a * x + b;
    }
};

//A bounding check for precenting negative numbers applies a<0 return 0
inline double clipNegative(double a){
    if(a<0){
        return 0;
    }
    return a;
}

#endif