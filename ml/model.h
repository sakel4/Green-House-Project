#pragma once
#include <cstdarg>
namespace Eloquent
{
    namespace ML
    {
        namespace Port
        {
            class DecisionTree
            {
            public:
                /**
                 * Predict class for features vector
                 */
                int predict(float *x)
                {
                    if (x[0] <= -1.3098707795143127)
                    {
                        if (x[1] <= -0.6233833283185959)
                        {
                            if (x[0] <= -1.5609248876571655)
                            {
                                return 0;
                            }

                            else
                            {
                                return 1;
                            }
                        }

                        else
                        {
                            return 0;
                        }
                    }

                    else
                    {
                        if (x[1] <= 1.4886309504508972)
                        {
                            return 1;
                        }

                        else
                        {
                            if (x[0] <= 0.017129480838775635)
                            {
                                return 0;
                            }

                            else
                            {
                                return 1;
                            }
                        }
                    }
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
                        return "Partial";
                    case 1:
                        return "Full";
                    default:
                        return "Houston we have a problem";
                    }
                }

            protected:
            };
        }
    }
}