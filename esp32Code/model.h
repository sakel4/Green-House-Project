#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class SVM {
            public:
                /**
                * Predict class for features vector
                */
                int predict(float* x) {
                    float kernels[4] = { 0 };
                    float decisions[1] = { 0 };
                    int votes[2] = { 0 };
                    kernels[0] = compute_kernel(x, 34.0, 57.0);
                    kernels[1] = compute_kernel(x, 28.0, 48.0);
                    kernels[2] = compute_kernel(x, 34.0, 54.0);
                    kernels[3] = compute_kernel(x, 32.0, 52.0);
                    float decision = 2.996123186217;
                    decision = decision - (+kernels[0] * -0.555432482102 + kernels[1] * -0.499969231637);
                    decision = decision - (+kernels[2] * 0.055401713739 + kernels[3] * 1.0);

                    return decision > 0 ? 0 : 1;
                }

                /**
                * Predict readable class name
                */
                const char* predictLabel(float* x) {
                    return idxToLabel(predict(x));
                }

                /**
                * Convert class idx to readable name
                */
                const char* idxToLabel(uint8_t classIdx) {
                    switch (classIdx) {
                    case 0:
                        return "full";
                    case 1:
                        return "middle";
                    default:
                        return "Houston we have a problem";
                    }
                }

            protected:
                /**
                * Compute kernel between feature vector and support vector.
                * Kernel type: linear
                */
                float compute_kernel(float* x, ...) {
                    va_list w;
                    va_start(w, 2);
                    float kernel = 0.0;

                    for (uint16_t i = 0; i < 2; i++) {
                        kernel += x[i] * va_arg(w, double);
                    }

                    return kernel;
                }
            };
        }
    }
}