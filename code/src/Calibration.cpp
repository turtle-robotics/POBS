#include <Hardware.h>
#include <functional>

class SubProfile{
    public:
    struct Expression{
        virtual double evaluate(double x);
    };

    struct QuadraticExpression : Expression{
        //ax^2+bx+c=y
        double a;
        double b;
        double c;

        QuadraticExpression(double a,double b,double c):a(a),b(b),c(c){}

        double evaluate(double x){
            return a*x*x+b*x+c;
        }
    };

    struct CheckedQuadraticExpression : QuadraticExpression{
        //The bound can not be zero;
        std::function<double(double)> check;

        CheckedQuadraticExpression(double a,double b,double c,std::function<double(double)> check): QuadraticExpression(a,b,c),check(check) {}

        double evaluate(double x){
            double eval=a*x*x+b*x+c;
            return check(eval);
        }
    };

    struct LinearExpression : Expression{
        //ax+b=y
        double a;
        double b;

        LinearExpression(double a,double b):a(a),b(b){}

        double evaluate(double x){
            return a*x+b;
        }
    };

    //takes in a thrust and returns the duty needed to get it.
    struct CheckedQuadraticExpression forward_thrust_profile_from_force={Forward_Thrust_Profile_From_Force
        ,[](double in){return in<Max_Forward_Thrust_ums?in:Max_Forward_Thrust_ums;}};   
      struct CheckedQuadraticExpression reverse_thrust_profile_from_force={Reverse_Thrust_Profile_From_Force
        ,[](double in){return in<Max_Reverse_Thrust_ums?in:Max_Reverse_Thrust_ums;}}; 

    struct QuadraticExpression forward_thrust_profile_from_microseconds ={Forward_Thrust_Profile_From_Microseconds}; //Todo calculate expression
    struct QuadraticExpression reverse_thrust_profile_from_microseconds ={Reverse_Thrust_Profile_From_Microseconds};
};


void* startCalibration(void* args){

}

double calibrateBuoyancy(){

}