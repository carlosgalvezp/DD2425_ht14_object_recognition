#ifndef BAGOFWORDS_H
#define BAGOFWORDS_H

#include <string>

class BagOfWords
{
public:
    BagOfWords();

    void train();

    std::string estimate_class();
};

#endif // BAGOFWORDS_H
