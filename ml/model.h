#pragma once
#include <cstdarg>
namespace Eloquent
{
    namespace ML
    {
        namespace Port
        {
            class SVM
            {
            public:
                /**
                 * Predict class for features vector
                 */
                int predict(float *x)
                {
                    float kernels[17] = {0};
                    float decisions[1] = {0};
                    int votes[2] = {0};
                    kernels[0] = compute_kernel(x, -1.728294255646, 1.104628346919);
                    kernels[1] = compute_kernel(x, -1.321825711938, -0.354581496348);
                    kernels[2] = compute_kernel(x, -1.800023998654, -1.813791339614);
                    kernels[3] = compute_kernel(x, -1.345735626273, 1.565431455319);
                    kernels[4] = compute_kernel(x, -1.967393399004, 0.874226792719);
                    kernels[5] = compute_kernel(x, -1.345735626273, 0.490224202386);
                    kernels[6] = compute_kernel(x, -1.393555454945, 1.488630937252);
                    kernels[7] = compute_kernel(x, -1.967393399004, 0.951027310786);
                    kernels[8] = compute_kernel(x, -1.25009596893, -0.354581496348);
                    kernels[9] = compute_kernel(x, -1.25009596893, -0.354581496348);
                    kernels[10] = compute_kernel(x, 0.184498891218, 0.797426274652);
                    kernels[11] = compute_kernel(x, 1.260445036329, 2.717439226319);
                    kernels[12] = compute_kernel(x, -1.274005883266, -0.508182532481);
                    kernels[13] = compute_kernel(x, -1.226186054594, -0.431382014414);
                    kernels[14] = compute_kernel(x, -1.297915797602, -0.508182532481);
                    kernels[15] = compute_kernel(x, -1.08272656858, -0.354581496348);
                    kernels[16] = compute_kernel(x, -1.25009596893, -0.354581496348);
                    float decision = 0.465146977737;
                    decision = decision - (+kernels[0] * -1.0 + kernels[1] * -1.0 + kernels[2] * -1.0 + kernels[3] * -0.968832662213 + kernels[4] * -1.0 + kernels[5] * -1.0 + kernels[6] * -1.0 + kernels[7] * -1.0);
                    decision = decision - (+kernels[8] * 1.0 + kernels[9] * 1.0 + kernels[10] * 0.895126690895 + kernels[11] * 1.0 + kernels[12] * 1.0 + kernels[13] * 1.0 + kernels[14] * 1.0 + kernels[15] * 0.073705971317 + kernels[16] * 1.0);

                    return decision > 0 ? 0 : 1;
                }

                /**
                 * Predict readable class name
                 */
                const char *predictLabel(float *x)
                {
                    return idxToLabel(predict(x));
                }

                /**
                 * Convert class idx to readable name
                 */
                const char *idxToLabel(uint8_t classIdx)
                {
                    switch (classIdx)
                    {
                    case 0:
                        return "Full";
                    case 1:
                        return "Partial";
                    default:
                        return "Houston we have a problem";
                    }
                }

            protected:
                /**
                 * Compute kernel between feature vector and support vector.
                 * Kernel type: rbf
                 */
                float compute_kernel(float *x, ...)
                {
                    va_list w;
                    va_start(w, 2);
                    float kernel = 0.0;

                    for (uint16_t i = 0; i < 2; i++)
                    {
                        kernel += pow(x[i] - va_arg(w, double), 2);
                    }

                    return exp(-0.1 * kernel);
                }
            };
        }
    }
}