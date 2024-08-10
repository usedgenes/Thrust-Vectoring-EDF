#include "CustomMath.h"

bool CustomMath::ComputeDelta(float _list[], int _size, int16_t *_delta) {

    if (_size <= 2)
        return false;

    int16_t maxVal = _list[0];
    int16_t minVal = _list[0];

    // CustomSerialPrint::println(F("Compute Delta : Samples: "));
    for (int sample = 0; sample < _size; sample++) {
        // CustomSerialPrint::print(_list[sample]);
        // CustomSerialPrint::print("\t");
        if (_list[sample] > maxVal)
            maxVal = _list[sample];
        if (_list[sample] < minVal)
            minVal = _list[sample];
    }
    // CustomSerialPrint::println("");
    (*_delta) = abs(maxVal - minVal);
    // CustomSerialPrint::print(F("Compute Delta: delta: "));
    // CustomSerialPrint::println(*_delta);
    return true;
}

bool CustomMath::ComputeMean(float _list[], int _size, int16_t _deltaThreshold, float *_mean) {
    if (_size <= 0)
        return false;

    // Check delta before computing mean
    int16_t delta = 0;
    if (!ComputeDelta(_list, _size, &delta)) {
        CustomSerialPrint::println(F("Compute Mean: Error, ComputeDelta failed"));
        return false;
    }
    if (delta > _deltaThreshold) {
        CustomSerialPrint::println(F("Compute Mean: Excessive delta:"));
        CustomSerialPrint::println(F("delta:"));
        CustomSerialPrint::println(delta);
        CustomSerialPrint::println(F("Threshold:"));
        CustomSerialPrint::println(_deltaThreshold);
        return false;
    }

    // Compute mean
    (*_mean) = 0;
    for (int sample = 0; sample < _size; sample++) {
        (*_mean) = (*_mean) + _list[sample];
    }
    (*_mean) = (*_mean) / _size;

    return true;
}

void CustomMath::VectorNormalize(float _vectorIn[], const int vectorSize) {
    float sumSquares = 0.0;
    for (int index = 0; index < vectorSize; index++)
        sumSquares += _vectorIn[index] * _vectorIn[index];

    float norm = sqrt(sumSquares);

    if (norm > 0.0)
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = _vectorIn[index] / norm;
    else
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = 0.0;
}
