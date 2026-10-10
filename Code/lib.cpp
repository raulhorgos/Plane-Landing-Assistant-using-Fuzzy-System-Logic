#include <Arduino.h>  // For min() and max()
#include "lib.h"

FuzzyLib::FuzzyLib() {
    numHeightMFs = 0;
    numAngleMFs = 0;
    numAdjAngleMFs = 0;
}

void FuzzyLib::setInputMFs(float heightMFs[][4], int heightMFTypes[], float angleMFs[][4], int angleMFTypes[], int numHeightMFs, int numAngleMFs) {
    this->numHeightMFs = numHeightMFs;
    this->numAngleMFs = numAngleMFs;
    for (int i = 0; i < numHeightMFs; i++) {
        for (int j = 0; j < 4; j++) {
            this->heightMFs[i][j] = heightMFs[i][j];
        }
        this->heightMFTypes[i] = heightMFTypes[i];
    }
    for (int i = 0; i < numAngleMFs; i++) {
        for (int j = 0; j < 4; j++) {
            this->angleMFs[i][j] = angleMFs[i][j];
        }
        this->angleMFTypes[i] = angleMFTypes[i];
    }
}

void FuzzyLib::setOutputMFs(float adjAngleMFs[][4], int adjAngleMFTypes[], int numAdjAngleMFs) {
    this->numAdjAngleMFs = numAdjAngleMFs;
    for (int i = 0; i < numAdjAngleMFs; i++) {
        for (int j = 0; j < 4; j++) {
            this->adjAngleMFs[i][j] = adjAngleMFs[i][j];
        }
        this->adjAngleMFTypes[i] = adjAngleMFTypes[i];
    }
}

void FuzzyLib::setRules(int rules[][3]) {
    for (int i = 0; i < MAX_RULES; i++) {
        for (int j = 0; j < 3; j++) {
            this->rules[i][j] = rules[i][j];
        }
    }
}

float FuzzyLib::triMf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0;
    if (x == b) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

float FuzzyLib::trapMf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0;
    if (x >= b && x <= c) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

void FuzzyLib::fuzzify(float height, float angle) {
    for (int i = 0; i < numHeightMFs; i++) {
        if (heightMFTypes[i] == 0) { // Triangular
            heightDegrees[i] = triMf(height, heightMFs[i][0], heightMFs[i][1], heightMFs[i][2]);
        } else { // Trapezoidal
            heightDegrees[i] = trapMf(height, heightMFs[i][0], heightMFs[i][1], heightMFs[i][2], heightMFs[i][3]);
        }
    }
    for (int i = 0; i < numAngleMFs; i++) {
        if (angleMFTypes[i] == 0) { // Triangular
            angleDegrees[i] = triMf(angle, angleMFs[i][0], angleMFs[i][1], angleMFs[i][2]);
        } else { // Trapezoidal
            angleDegrees[i] = trapMf(angle, angleMFs[i][0], angleMFs[i][1], angleMFs[i][2], angleMFs[i][3]);
        }
    }
}

void FuzzyLib::evaluateRules() {
    for (int i = 0; i < MAX_RULES; i++) {
        int heightIdx = rules[i][0];
        int angleIdx = rules[i][1];
        if (heightIdx == -1 || angleIdx == -1) {
            ruleFiringDegrees[i] = 0.0;
            continue;
        }
        // Use min (AND) for rule firing degree
        ruleFiringDegrees[i] = min(heightDegrees[heightIdx], angleDegrees[angleIdx]);
    }
}

void FuzzyLib::aggregate(float* outputDegrees) {
    for (int i = 0; i < numAdjAngleMFs; i++) {
        outputDegrees[i] = 0.0;
    }
    for (int i = 0; i < MAX_RULES; i++) {
        int adjAngleIdx = rules[i][2];
        if (adjAngleIdx != -1 && ruleFiringDegrees[i] > 0) {
            outputDegrees[adjAngleIdx] = max(outputDegrees[adjAngleIdx], ruleFiringDegrees[i]);
        }
    }
}

float FuzzyLib::defuzzify() {
    float outputDegrees[MAX_MF];
    aggregate(outputDegrees);
    
    // Use Center of Area (COA) defuzzification
    float numerator = 0.0;
    float denominator = 0.0;
    const int samples = 1000;
    float step = 100.0 / (samples - 1); // Adjustment Angle range [0, 100]
    
    for (int i = 0; i < samples; i++) {
        float x = i * step;
        float membership = 0.0;
        for (int j = 0; j < numAdjAngleMFs; j++) {
            float degree;
            if (adjAngleMFTypes[j] == 0) { // Triangular
                degree = min(outputDegrees[j], triMf(x, adjAngleMFs[j][0], adjAngleMFs[j][1], adjAngleMFs[j][2]));
            } else { // Trapezoidal
                degree = min(outputDegrees[j], trapMf(x, adjAngleMFs[j][0], adjAngleMFs[j][1], adjAngleMFs[j][2], adjAngleMFs[j][3]));
            }
            membership = max(membership, degree);
        }
        numerator += x * membership;
        denominator += membership;
    }
    
    if (denominator == 0) return 0.0;
    return numerator / denominator;
}