#include <vector>
#include <cmath>
class LowPassFilter {
public:
    LowPassFilter(){}
    void init(double cutoffFrequency, double deltaTime) {
        alpha = cutoffFrequency * deltaTime / (1 + cutoffFrequency * deltaTime);
        output = 0;
    }

    double filter(double input) {
        output = alpha * input + (1.0 - alpha) * output;
        return output;
    }

private:
    double alpha;
    double output;
};