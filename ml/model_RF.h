#pragma once
#include <cstdarg>
namespace Eloquent
{
    namespace ML
    {
        namespace Port
        {
            class RandomForest
            {
            public:
                /**
                 * Predict class for features vector
                 */
                int predict(float *x)
                {
                    uint8_t votes[2] = {0};
                    // tree #1
                    if (x[0] <= -1.3098707795143127)
                    {
                        if (x[1] <= -0.6233833283185959)
                        {
                            if (x[0] <= -1.5609248876571655)
                            {
                                votes[0] += 1;
                            }

                            else
                            {
                                votes[1] += 1;
                            }
                        }

                        else
                        {
                            votes[0] += 1;
                        }
                    }

                    else
                    {
                        votes[1] += 1;
                    }

                    // tree #2
                    if (x[1] <= 0.8358265459537506)
                    {
                        if (x[1] <= -1.7753911018371582)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            if (x[1] <= 0.4518239498138428)
                            {
                                votes[1] += 1;
                            }

                            else
                            {
                                if (x[1] <= 0.5286244601011276)
                                {
                                    if (x[0] <= -0.32956430315971375)
                                    {
                                        votes[0] += 1;
                                    }

                                    else
                                    {
                                        votes[1] += 1;
                                    }
                                }

                                else
                                {
                                    votes[1] += 1;
                                }
                            }
                        }
                    }

                    else
                    {
                        if (x[1] <= 2.6406387090682983)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            if (x[1] <= 2.755839467048645)
                            {
                                votes[1] += 1;
                            }

                            else
                            {
                                votes[0] += 1;
                            }
                        }
                    }

                    // tree #3
                    if (x[1] <= 0.4518239498138428)
                    {
                        if (x[1] <= -1.7753911018371582)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            if (x[1] <= -0.3161812424659729)
                            {
                                if (x[1] <= -0.39298175275325775)
                                {
                                    votes[1] += 1;
                                }

                                else
                                {
                                    if (x[0] <= -1.285960853099823)
                                    {
                                        votes[0] += 1;
                                    }

                                    else
                                    {
                                        votes[1] += 1;
                                    }
                                }
                            }

                            else
                            {
                                votes[1] += 1;
                            }
                        }
                    }

                    else
                    {
                        if (x[0] <= -0.6164832599461079)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    // tree #4
                    if (x[0] <= -1.2979158163070679)
                    {
                        votes[0] += 1;
                    }

                    else
                    {
                        if (x[1] <= 1.5654314160346985)
                        {
                            votes[1] += 1;
                        }

                        else
                        {
                            votes[0] += 1;
                        }
                    }

                    // tree #5
                    if (x[1] <= 1.2198291420936584)
                    {
                        if (x[0] <= -1.3098707795143127)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    else
                    {
                        votes[0] += 1;
                    }

                    // tree #6
                    if (x[1] <= 1.0662280917167664)
                    {
                        if (x[1] <= -1.6985905766487122)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            if (x[1] <= -0.39298175275325775)
                            {
                                votes[1] += 1;
                            }

                            else
                            {
                                if (x[0] <= -1.285960853099823)
                                {
                                    votes[0] += 1;
                                }

                                else
                                {
                                    votes[1] += 1;
                                }
                            }
                        }
                    }

                    else
                    {
                        if (x[0] <= 0.005174517631530762)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    // tree #7
                    if (x[1] <= 0.7590260207653046)
                    {
                        if (x[1] <= -0.3161812424659729)
                        {
                            if (x[0] <= -1.3098707795143127)
                            {
                                if (x[1] <= -0.6233833283185959)
                                {
                                    votes[1] += 1;
                                }

                                else
                                {
                                    votes[0] += 1;
                                }
                            }

                            else
                            {
                                votes[1] += 1;
                            }
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    else
                    {
                        if (x[0] <= -0.2578345239162445)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    // tree #8
                    if (x[0] <= -1.3098707795143127)
                    {
                        if (x[0] <= -1.3337807059288025)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            if (x[1] <= -0.6233833283185959)
                            {
                                votes[1] += 1;
                            }

                            else
                            {
                                votes[0] += 1;
                            }
                        }
                    }

                    else
                    {
                        if (x[0] <= -1.2142310738563538)
                        {
                            if (x[0] <= -1.2381410002708435)
                            {
                                votes[1] += 1;
                            }

                            else
                            {
                                if (x[1] <= 0.6822254955768585)
                                {
                                    votes[1] += 1;
                                }

                                else
                                {
                                    votes[0] += 1;
                                }
                            }
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    // tree #9
                    if (x[1] <= 0.9126270711421967)
                    {
                        if (x[0] <= -1.3098707795143127)
                        {
                            votes[0] += 1;
                        }

                        else
                        {
                            votes[1] += 1;
                        }
                    }

                    else
                    {
                        if (x[1] <= 1.2966296672821045)
                        {
                            if (x[1] <= 1.1430286169052124)
                            {
                                votes[0] += 1;
                            }

                            else
                            {
                                votes[1] += 1;
                            }
                        }

                        else
                        {
                            votes[0] += 1;
                        }
                    }

                    // tree #10
                    if (x[0] <= -1.2979158163070679)
                    {
                        votes[0] += 1;
                    }

                    else
                    {
                        if (x[1] <= 1.6422319412231445)
                        {
                            votes[1] += 1;
                        }

                        else
                        {
                            if (x[0] <= 0.005174517631530762)
                            {
                                votes[0] += 1;
                            }

                            else
                            {
                                votes[1] += 1;
                            }
                        }
                    }

                    // return argmax of votes
                    uint8_t classIdx = 0;
                    float maxVotes = votes[0];

                    for (uint8_t i = 1; i < 2; i++)
                    {
                        if (votes[i] > maxVotes)
                        {
                            classIdx = i;
                            maxVotes = votes[i];
                        }
                    }

                    return classIdx;
                }

            protected:
            };
        }
    }
}