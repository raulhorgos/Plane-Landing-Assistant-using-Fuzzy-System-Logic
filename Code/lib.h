#ifndef FUZZY_LIB_H
#define FUZZY_LIB_H

#define MAX_MF 5      // Maximum number of membership functions per variable
#define MAX_RULES 9   // Maximum number of rules (3x3 for two inputs)

class FuzzyLib {
public:
    FuzzyLib();
    
    // Initialize membership functions for inputs and output
    void setInputMFs(float heightMFs[][4], int heightMFTypes[], float angleMFs[][4], int angleMFTypes[], int numHeightMFs, int numAngleMFs);
    void setOutputMFs(float adjAngleMFs[][4], int adjAngleMFTypes[], int numAdjAngleMFs);
    
    // Set the rules (3x3 rule base for two inputs)
    void setRules(int rules[][3]);
    
    // Fuzzify the inputs
    void fuzzify(float height, float angle);
    
    // Evaluate rules and compute firing degrees
    void evaluateRules();
    
    // Defuzzify to get the crisp Adjustment Angle output
    float defuzzify();
    
private:
    // Membership function arrays for Height, Angle, and Adjustment Angle
    float heightMFs[MAX_MF][4];  // Parameters: [a, b, c, d] for trapezoidal, [a, b, c, c] for triangular
    float angleMFs[MAX_MF][4];
    float adjAngleMFs[MAX_MF][4];
    int heightMFTypes[MAX_MF];   // 0 for triangular, 1 for trapezoidal
    int angleMFTypes[MAX_MF];
    int adjAngleMFTypes[MAX_MF];
    int numHeightMFs, numAngleMFs, numAdjAngleMFs;
    
    // Rule base: [heightMFIndex, angleMFIndex, adjAngleMFIndex]
    int rules[MAX_RULES][3];
    
    // Membership degrees for inputs
    float heightDegrees[MAX_MF];
    float angleDegrees[MAX_MF];
    
    // Firing degrees for rules
    float ruleFiringDegrees[MAX_RULES];
    
    // Compute membership degree for a triangular MF
    float triMf(float x, float a, float b, float c);
    
    // Compute membership degree for a trapezoidal MF
    float trapMf(float x, float a, float b, float c, float d);
    
    // Compute the aggregated output membership function
    void aggregate(float* outputDegrees);
};

#endif