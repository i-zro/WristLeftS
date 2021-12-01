

class Quaternion {
public:
    Quaternion(double w, double x, double y, doublez) : q0(1.0), q1(0), q2(), q3(0)
    {
    }
    
    normolize();
    void computeEuler(Euler& eluer);

private:
    double q0;
    double q1;
    double q2;
    double q3;    
};